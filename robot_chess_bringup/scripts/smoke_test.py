#!/usr/bin/env python3
"""End-to-end check of the running simulation (used by CI).

Plays 1. e4 e5 2. Nf3 through the referee - exactly like the player consoles
do - and checks that
  * every move is executed by the right robot and committed by the referee,
  * the moved pieces really stand on their destination squares in Gazebo,
  * the camera verification recognised the pieces.

    roslaunch robot_chess_bringup simulation.launch gui:=false rviz:=false
    rosrun robot_chess_bringup smoke_test.py
"""
import math
import sys

import rospy
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from robot_chess.board_geometry import BoardGeometry
from robot_chess_msgs.msg import GameState
from robot_chess_msgs.srv import SubmitMove

MOVES = [
    ("white", "e4", "white_pawn_e", "e4"),
    ("black", "e5", "black_pawn_e", "e5"),
    ("white", "Nf3", "white_knight_g", "f3"),
]
POSITION_TOLERANCE = 0.05


class SmokeTest(object):

    def __init__(self):
        self.events = []
        self.state = None
        rospy.Subscriber("/chess/events", String, self.on_event, queue_size=50)
        rospy.Subscriber("/chess/state", GameState, self.on_state, queue_size=5)
        rospy.wait_for_service("/chess/submit_move", timeout=600)
        rospy.wait_for_service("/gazebo/get_model_state", timeout=600)
        self.submit = rospy.ServiceProxy("/chess/submit_move", SubmitMove)
        self.model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        self.geometry = BoardGeometry.from_dict(rospy.get_param("/chess_referee/board"))

    def on_event(self, msg):
        self.events.append(msg.data)
        print("[event] %s" % msg.data)
        sys.stdout.flush()

    def on_state(self, msg):
        self.state = msg

    def play(self, color, move):
        deadline = rospy.Time.now() + rospy.Duration(600)
        while not rospy.is_shutdown():
            res = self.submit(color, move)
            if res.accepted:
                print("submitted %s %s: %s" % (color, move, res.message))
                break
            if rospy.Time.now() > deadline:
                raise RuntimeError("move %s never accepted: %s" % (move, res.message))
            print("not accepted yet (%s), retrying" % res.message)
            rospy.sleep(5.0)

        deadline = rospy.Time.now() + rospy.Duration(1200)
        while not rospy.is_shutdown():
            st = self.state
            if st is not None and st.move_history_san and st.move_history_san[-1] == move \
                    and st.status != GameState.STATUS_ROBOT_MOVING:
                return
            if st is not None and st.status == GameState.STATUS_ROBOT_ERROR:
                raise RuntimeError(st.status_text)
            if rospy.Time.now() > deadline:
                raise RuntimeError("timeout while the robot executed %s" % move)
            rospy.sleep(1.0)

    def check_position(self, model, square):
        p = self.model_state(model_name=model, relative_entity_name="world").pose.position
        x, y = self.geometry.square_center(square)
        err = math.hypot(p.x - x, p.y - y)
        print("%s at (%.3f, %.3f), %s centre (%.3f, %.3f), error %.3f m" % (
            model, p.x, p.y, square, x, y, err))
        if err > POSITION_TOLERANCE or p.z > 0.05:
            raise RuntimeError("%s is not on %s" % (model, square))

    def yaw_of(self, model):
        q = self.model_state(model_name=model, relative_entity_name="world").pose.orientation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def motion_check(self, robot):
        """Spin the robot briefly and compare its Gazebo ground truth yaw."""
        pub = rospy.Publisher("/%s/cmd_vel" % robot, Twist, queue_size=1)
        before = self.yaw_of(robot)
        cmd = Twist()
        cmd.angular.z = 0.5
        end = rospy.Time.now() + rospy.Duration(2.0)
        while rospy.Time.now() < end:
            pub.publish(cmd)
            rospy.sleep(0.05)
        pub.publish(Twist())
        rospy.sleep(0.5)
        turned = math.atan2(math.sin(self.yaw_of(robot) - before),
                            math.cos(self.yaw_of(robot) - before))
        print("motion check %s: commanded ~1.0 rad, Gazebo ground truth turned %.2f rad"
              % (robot, turned))
        if abs(turned) < 0.2:
            raise RuntimeError("%s does not move on cmd_vel" % robot)

    def run(self):
        for robot in ("white", "black"):
            self.motion_check(robot)
        for color, move, model, square in MOVES:
            start = rospy.Time.now()
            self.play(color, move)
            print("%s %s done in %.0f s (sim time)" % (color, move,
                                                       (rospy.Time.now() - start).to_sec()))
            self.check_position(model, square)
        ok = [e for e in self.events if "verify: OK" in e]
        bad = [e for e in self.events if "MISMATCH" in e]
        print("camera verifications: %d ok, %d mismatches" % (len(ok), len(bad)))
        if len(ok) < len(MOVES) or bad:
            raise RuntimeError("camera verification failed")
        print("SMOKE TEST PASSED")


if __name__ == "__main__":
    rospy.init_node("smoke_test")
    try:
        SmokeTest().run()
    except Exception as exc:  # report and fail the CI job
        print("SMOKE TEST FAILED: %s" % exc)
        sys.exit(1)
