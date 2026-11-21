#!/usr/bin/env python3
"""Moves chess pieces with the myCobot 280.

Action server  execute_move      robot_chess_msgs/ExecuteMove
Service        ~observe_pose     std_srvs/Trigger  (camera above the board)

Every PieceTask becomes
    above(from) -> down -> close gripper -> up -> above(to) -> down
    -> open gripper -> up
with the tool pointing straight down.  Straight tool paths are sampled
every `cartesian_step` metres, each sample is solved with the IK
(robot_chess.kinematics, seeded with the previous solution so the arm moves
smoothly) and the whole path is sent as one joint trajectory, timed so that
no joint exceeds `speed` rad/s.  After the move the arm goes back to the
observe pose so the camera can check the result.

Arm and gripper are driven through control_msgs/FollowJointTrajectory:
in Gazebo by the ros_control controllers, on the real robot by
mycobot_driver.py - this node is the same in both cases.

~base/stations: forward offsets [m] the LIMO may drive to when a square is
out of reach from home (robot_chess.stations); [0.0] = fixed base.  The
base is driven straight with cmd_vel and odometry feedback, always with the
arm lifted, and returns home before the camera looks at the board.

~virtual_grasp (simulation): after closing / before opening the gripper the
piece is attached / released by the simulation (/chess_sim/gripper/...).
On the real robot a promotion asks the operator to exchange the piece
(/chess/operator, confirm with  rosservice call /arm_executor/confirm).
"""
import math
import threading

import actionlib
import numpy as np
import rospy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from robot_chess.board_geometry import BoardGeometry
from robot_chess.kinematics import JOINT_NAMES, MyCobotKinematics, interpolate_line
from robot_chess.stations import choose_station
from robot_chess_msgs.msg import ExecuteMoveAction, ExecuteMoveFeedback, ExecuteMoveResult
from robot_chess_msgs.srv import ReplacePiece

FINGERS = ["finger_left_joint", "finger_right_joint"]


class MoveFailed(Exception):
    pass


class ArmExecutor(object):

    def __init__(self):
        self.g = BoardGeometry.from_dict(rospy.get_param("~board", {}))
        arm = rospy.get_param("~arm", {})
        self.kin = MyCobotKinematics(tcp_offset=arm.get("tcp_offset", 0.06))
        self.speed = arm.get("speed", 0.6)
        self.step = arm.get("cartesian_step", 0.01)
        self.observe_yaw = arm.get("observe_yaw", 0.0)
        self.gripper_open = arm.get("gripper_open", 0.012)
        self.gripper_closed = arm.get("gripper_closed", 0.004)
        self.virtual_grasp = rospy.get_param("~virtual_grasp", False)
        base = rospy.get_param("~base", {})
        self.stations = [float(o) for o in base.get("stations", [0.0])]
        self.base_speed = base.get("speed", 0.05)
        self.odom = None
        if len(self.stations) > 1:
            self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
            rospy.Subscriber("odom", Odometry, self.on_odom, queue_size=1)

        self.q = None
        self.q_lock = threading.Lock()
        rospy.Subscriber("joint_states", JointState, self.on_joints, queue_size=1)
        self.operator_pub = rospy.Publisher("/chess/operator", String, queue_size=5, latch=True)
        self.confirmed = threading.Event()

        self.arm = actionlib.SimpleActionClient("arm_controller/follow_joint_trajectory",
                                                FollowJointTrajectoryAction)
        self.gripper = actionlib.SimpleActionClient("gripper_controller/follow_joint_trajectory",
                                                    FollowJointTrajectoryAction)
        rospy.loginfo("waiting for the arm and gripper controllers")
        self.arm.wait_for_server()
        self.gripper.wait_for_server()
        while self.current_q() is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

        if self.virtual_grasp:
            for name in ("gripper/attach", "gripper/release", "replace"):
                rospy.wait_for_service("/chess_sim/" + name)
            self.attach = rospy.ServiceProxy("/chess_sim/gripper/attach", Trigger)
            self.release = rospy.ServiceProxy("/chess_sim/gripper/release", Trigger)
            self.replace = rospy.ServiceProxy("/chess_sim/replace", ReplacePiece)

        rospy.Service("~observe_pose", Trigger, self.on_observe)
        rospy.Service("~confirm", Trigger, self.on_confirm)
        self.server = actionlib.SimpleActionServer("execute_move", ExecuteMoveAction,
                                                   execute_cb=self.execute, auto_start=False)
        self.server.start()
        rospy.loginfo("arm executor ready")

    # ------------------------------------------------------------- state
    def on_joints(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if all(n in pos for n in JOINT_NAMES):
            with self.q_lock:
                self.q = [pos[n] for n in JOINT_NAMES]

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.odom = (p.x, p.y)

    def current_q(self):
        with self.q_lock:
            return None if self.q is None else list(self.q)

    # ------------------------------------------------------------ motion
    def _send(self, client, names, points, what):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = JointTrajectory(joint_names=names, points=points)
        client.send_goal(goal)
        duration = points[-1].time_from_start.to_sec() if points else 0.0
        if not client.wait_for_result(rospy.Duration(duration + 15.0)):
            client.cancel_goal()
            raise MoveFailed("%s: trajectory timed out" % what)
        result = client.get_result()
        if result is not None and result.error_code != 0:
            raise MoveFailed("%s: controller error %d %s" % (
                what, result.error_code, result.error_string))

    def _timed(self, path, start):
        """Joint trajectory points along `path`, limited to self.speed."""
        points, t, prev = [], 0.0, np.array(start)
        for q in path:
            q = np.array(q)
            dt = max(float(np.abs(q - prev).max()) / self.speed, 0.05)
            t += dt
            points.append(JointTrajectoryPoint(positions=list(q),
                                               time_from_start=rospy.Duration(t)))
            prev = q
        return points

    def move_joints(self, target, what="move"):
        start = self.current_q()
        n = max(1, int(math.ceil(np.abs(np.array(target) - np.array(start)).max() / 0.1)))
        path = [list(np.array(start) + (np.array(target) - np.array(start)) * i / n)
                for i in range(1, n + 1)]
        self._send(self.arm, JOINT_NAMES, self._timed(path, start), what)

    def move_line(self, target, what, yaw=None):
        """Straight TCP motion from the current position to `target`."""
        start = self.current_q()
        tcp, _ = self.kin.tcp(start)
        path, seed = [], start
        for p in interpolate_line(tcp, target, self.step):
            q = self.kin.ik_down(p, seed=seed, yaw=yaw)
            if q is None:
                raise MoveFailed("%s: (%.3f, %.3f, %.3f) is out of reach" % (what, p[0], p[1], p[2]))
            # keep joint 6 from spinning: stay near the previous solution
            if abs(q[5] - seed[5]) > math.pi:
                q[5] += -2 * math.pi if q[5] > seed[5] else 2 * math.pi
            path.append(q)
            seed = q
        self._send(self.arm, JOINT_NAMES, self._timed(path, start), what)

    def set_gripper(self, opening, what):
        pt = JointTrajectoryPoint(positions=[opening, opening],
                                  time_from_start=rospy.Duration(0.8))
        self._send(self.gripper, FINGERS, [pt], what)

    def go_observe(self):
        if len(self.stations) > 1:
            self.drive_to_station(0.0)
        q = self.kin.ik_down(self.g.observe_point(), seed=self.current_q(), yaw=self.observe_yaw)
        if q is None:
            q = self.kin.ik_down(self.g.observe_point(), yaw=self.observe_yaw)
        if q is None:
            raise MoveFailed("observe pose out of reach")
        tcp, _ = self.kin.tcp(self.current_q())
        if tcp[2] < self.g.board_z + self.g.travel_height - 0.005:
            # first lift straight up, away from the pieces
            self.move_line([tcp[0], tcp[1], self.g.board_z + self.g.travel_height], "lift")
        self.move_joints(q, "observe pose")

    # ------------------------------------------------------------- base
    def drive_to_station(self, offset):
        """Drive straight forward/backward until the robot is `offset` metres
        from its home position (odometry feedback)."""
        delta = offset - self.g.base_offset
        if abs(delta) < 1e-4:
            return
        if self.odom is None:
            raise MoveFailed("no odometry - cannot move the base")
        tcp, _ = self.kin.tcp(self.current_q())
        if tcp[2] < self.g.board_z + self.g.travel_height - 0.005:
            self.move_line([tcp[0], tcp[1], self.g.board_z + self.g.travel_height], "lift")
        start = self.odom
        rate = rospy.Rate(30)
        deadline = rospy.Time.now() + rospy.Duration(10.0 + abs(delta) / 0.01)
        while not rospy.is_shutdown():
            done = math.hypot(self.odom[0] - start[0], self.odom[1] - start[1])
            left = abs(delta) - done
            if left < 0.002:
                break
            if rospy.Time.now() > deadline:
                self.cmd_pub.publish(Twist())
                raise MoveFailed("base did not reach station %.3f" % offset)
            cmd = Twist()
            cmd.linear.x = math.copysign(min(self.base_speed, max(0.01, 2.0 * left)), delta)
            self.cmd_pub.publish(cmd)
            rate.sleep()
        self.cmd_pub.publish(Twist())
        rospy.sleep(0.3)
        self.g.base_offset = offset
        rospy.loginfo("base at station %+.3f m", offset)

    def ensure_reachable(self, label):
        if len(self.stations) <= 1:
            return
        offset = choose_station(self.g, self.kin, label, self.stations, self.g.base_offset)
        if offset is None:
            raise MoveFailed("%s is out of reach from every base station" % label)
        self.drive_to_station(offset)

    # ------------------------------------------------------- pick & place
    def pick(self, label):
        self.ensure_reachable(label)
        self.set_gripper(self.gripper_open, "open")
        self.move_line(self.g.travel_point(label), "above %s" % label)
        self.move_line(self.g.grasp_point(label), "down to %s" % label)
        self.set_gripper(self.gripper_closed, "close")
        if self.virtual_grasp:
            res = self.attach()
            if not res.success:
                raise MoveFailed("grasp at %s failed: %s" % (label, res.message))
        self.move_line(self.g.travel_point(label), "lift from %s" % label)

    def place(self, label):
        self.ensure_reachable(label)
        self.move_line(self.g.travel_point(label), "above %s" % label)
        self.move_line(self.g.grasp_point(label), "down to %s" % label)
        if self.virtual_grasp:
            self.release()
        self.set_gripper(self.gripper_open, "open")
        self.move_line(self.g.travel_point(label), "lift from %s" % label)

    def promote(self, task):
        if self.virtual_grasp:
            self.move_line(self.g.grasp_point(task.to_label), "promotion")
            res = self.replace(task.piece_color, task.promote_to)
            self.move_line(self.g.travel_point(task.to_label), "lift")
            if not res.success:
                raise MoveFailed("promotion failed: %s" % res.message)
            return
        self.confirmed.clear()
        text = "Please replace the pawn on %s by a %s %s, then confirm" % (
            task.to_label, task.piece_color, task.promote_to)
        self.operator_pub.publish(String(text))
        rospy.logwarn("OPERATOR: %s (rosservice call %s/confirm)", text, rospy.get_name())
        while not self.confirmed.wait(0.5):
            if rospy.is_shutdown() or self.server.is_preempt_requested():
                raise MoveFailed("promotion not confirmed")
        self.operator_pub.publish(String(""))

    # ------------------------------------------------------------ servers
    def feedback(self, index, stage, text):
        rospy.loginfo("task %d %s: %s", index, stage, text)
        self.server.publish_feedback(ExecuteMoveFeedback(task_index=index, stage=stage, text=text))

    def execute(self, goal):
        try:
            for i, task in enumerate(goal.tasks):
                self.feedback(i, "pick", "%s %s on %s" % (task.piece_color, task.piece_type,
                                                          task.from_label))
                self.pick(task.from_label)
                self.feedback(i, "place", "to %s" % task.to_label)
                self.place(task.to_label)
                if task.promote_to:
                    self.feedback(i, "promote", task.promote_to)
                    self.promote(task)
            self.feedback(len(goal.tasks), "observe", "back to the observe pose")
            self.go_observe()
        except MoveFailed as exc:
            rospy.logerr("move %s failed: %s", goal.move_uci, exc)
            self.server.set_aborted(ExecuteMoveResult(False, str(exc)))
            return
        self.server.set_succeeded(ExecuteMoveResult(True, "%s done" % goal.move_uci))

    def on_observe(self, _req):
        try:
            self.go_observe()
        except MoveFailed as exc:
            return TriggerResponse(False, str(exc))
        return TriggerResponse(True, "in observe pose")

    def on_confirm(self, _req):
        self.confirmed.set()
        return TriggerResponse(True, "confirmed")


if __name__ == "__main__":
    rospy.init_node("arm_executor")
    ArmExecutor()
    rospy.spin()
