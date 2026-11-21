#!/usr/bin/env python3
"""Executes chess moves physically with one LIMO PRO.

Action server  execute_move  (robot_chess_msgs/ExecuteMove)

For every PieceTask of the goal:
    1. approach  - choose a collision-free pose next to the piece
                   (BoardGeometry.approach_poses) and drive there with move_base;
                   if navigation fails, the next candidate pose is tried
    2. align     - turn in place until the camera looks straight at the piece
    3. verify    - ask the camera (verify_piece) whether the expected piece is there
    4. pick      - carrier attach (simulation: piece goes on the robot plate,
                   real robot: an assistant is asked to load it)
    5. transport - approach the destination the same way
    6. place     - carrier release (+ replace for a pawn promotion)
After the last task the robot drives back to its home pose.

Parameters
    ~robot_name        name used by the carrier (Gazebo model name)
    ~color             "white" | "black"  (selects the home pose)
    ~board, ~home      board geometry / home poses (board yaml)
    ~base_frame        robot base frame (e.g. white/base_footprint)
    ~global_frame      map
    ~carrier_ns        namespace of the carrier services (/chess_carrier)
    ~verify_with_camera, ~abort_on_mismatch
    ~navigation_timeout [s]
"""
import math
import re

import actionlib
import rospy
import tf2_ros
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from robot_chess.board_geometry import BoardGeometry, normalize_angle
from robot_chess_msgs.msg import (ExecuteMoveAction, ExecuteMoveFeedback,
                                  ExecuteMoveResult)
from robot_chess_msgs.srv import (AttachPiece, ReleasePiece, ReplacePiece,
                                  VerifyPiece)

GRAVEYARD_RE = re.compile(r"graveyard-(white|black)-(\d+)")


class MoveAborted(Exception):
    pass


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PieceMover(object):

    def __init__(self):
        ns = rospy.get_namespace().strip("/")
        self.robot_name = rospy.get_param("~robot_name", ns or "limo")
        self.color = rospy.get_param("~color", "white")
        self.geometry = BoardGeometry.from_dict(rospy.get_param("~board", {}))
        home = rospy.get_param("~home/" + self.color,
                               {"x": 0.0, "y": -3.9, "yaw": math.pi / 2})
        hx, hy = self.geometry.board_to_map(home["x"], home["y"])
        self.home = (hx, hy, self.geometry.board_yaw_to_map(home["yaw"]))
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")
        self.global_frame = rospy.get_param("~global_frame", "map")
        self.verify = rospy.get_param("~verify_with_camera", True)
        self.abort_on_mismatch = rospy.get_param("~abort_on_mismatch", False)
        self.nav_timeout = rospy.get_param("~navigation_timeout", 180.0)
        self.max_candidates = rospy.get_param("~max_approach_candidates", 4)
        self.return_home = rospy.get_param("~return_home", True)
        carrier = rospy.get_param("~carrier_ns", "/chess_carrier")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("[%s] waiting for move_base", self.robot_name)
        self.move_base.wait_for_server()

        rospy.loginfo("[%s] waiting for carrier services in %s", self.robot_name, carrier)
        for name in ("attach", "release", "replace"):
            rospy.wait_for_service(carrier + "/" + name)
        self.attach = rospy.ServiceProxy(carrier + "/attach", AttachPiece)
        self.release = rospy.ServiceProxy(carrier + "/release", ReleasePiece)
        self.replace = rospy.ServiceProxy(carrier + "/replace", ReplacePiece)
        self.verify_srv = rospy.ServiceProxy("verify_piece", VerifyPiece)

        self.server = actionlib.SimpleActionServer(
            "execute_move", ExecuteMoveAction, execute_cb=self.execute, auto_start=False)
        self.server.start()
        rospy.loginfo("[%s] piece mover ready", self.robot_name)

    # ================================================================ helpers
    def feedback(self, index, stage, text):
        rospy.loginfo("[%s] task %d %s: %s", self.robot_name, index, stage, text)
        self.server.publish_feedback(ExecuteMoveFeedback(task_index=index, stage=stage,
                                                         text=text))

    def check_preempt(self):
        if self.server.is_preempt_requested() or rospy.is_shutdown():
            self.move_base.cancel_all_goals()
            self.stop()
            raise MoveAborted("preempted")

    def robot_pose(self):
        t = self.tf_buffer.lookup_transform(self.global_frame, self.base_frame,
                                            rospy.Time(0), rospy.Duration(2.0))
        tr = t.transform
        return (tr.translation.x, tr.translation.y, yaw_from_quaternion(tr.rotation))

    def stop(self):
        self.cmd_pub.publish(Twist())

    def navigate(self, x, y, yaw):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.global_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.target_pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.move_base.send_goal(goal)
        deadline = rospy.Time.now() + rospy.Duration(self.nav_timeout)
        rate = rospy.Rate(5)
        while not self.move_base.wait_for_result(rospy.Duration(0.01)):
            self.check_preempt()
            if rospy.Time.now() > deadline:
                self.move_base.cancel_goal()
                return False
            rate.sleep()
        return self.move_base.get_state() == GoalStatus.SUCCEEDED

    def face_point(self, tx, ty, tolerance=0.03, timeout=10.0):
        """Rotate in place until the robot (and its camera) faces (tx, ty)."""
        rate = rospy.Rate(20)
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        while rospy.Time.now() < deadline:
            self.check_preempt()
            x, y, yaw = self.robot_pose()
            err = normalize_angle(math.atan2(ty - y, tx - x) - yaw)
            if abs(err) < tolerance:
                break
            cmd = Twist()
            w = max(-0.6, min(0.6, 1.5 * err))
            cmd.angular.z = math.copysign(max(abs(w), 0.15), w)
            self.cmd_pub.publish(cmd)
            rate.sleep()
        self.stop()

    def approach(self, index, target_xy, obstacles, label):
        """Drive to a pose from which the piece at target_xy can be handled."""
        self.check_preempt()
        try:
            rxy = self.robot_pose()[:2]
        except tf2_ros.TransformException:
            rxy = None
        candidates = self.geometry.approach_poses(target_xy, obstacles, rxy)
        if not candidates:
            raise MoveAborted("no free approach pose around %s" % label)
        for i, (x, y, yaw) in enumerate(candidates[:self.max_candidates]):
            self.feedback(index, "approach", "%s via (%.2f, %.2f) [candidate %d]" % (
                label, x, y, i + 1))
            if self.navigate(x, y, yaw):
                self.face_point(*target_xy)
                return
            rospy.logwarn("[%s] navigation to candidate %d for %s failed",
                          self.robot_name, i + 1, label)
        raise MoveAborted("could not reach %s" % label)

    # ============================================================== execution
    @staticmethod
    def _obstacle_points(occupied, graveyard_used):
        return list(occupied.values()) + [p for p in graveyard_used.values()]

    def execute(self, goal):
        occupied = {sq: self.geometry.square_center(sq) for sq in goal.occupied_squares}
        graveyard_used = {}
        try:
            for index, task in enumerate(goal.tasks):
                self.run_task(index, task, occupied, graveyard_used)
            if self.return_home:
                self.feedback(len(goal.tasks), "return", "driving home")
                if not self.navigate(*self.home):
                    rospy.logwarn("[%s] could not reach the home pose", self.robot_name)
        except MoveAborted as exc:
            self.stop()
            result = ExecuteMoveResult(success=False, message=str(exc))
            if self.server.is_preempt_requested():
                self.server.set_preempted(result)
            else:
                self.server.set_aborted(result)
            return
        except (rospy.ServiceException, tf2_ros.TransformException) as exc:
            self.stop()
            self.server.set_aborted(ExecuteMoveResult(success=False, message=str(exc)))
            return
        self.server.set_succeeded(ExecuteMoveResult(success=True,
                                                    message="move %s done" % goal.move_uci))

    def run_task(self, index, task, occupied, graveyard_used):
        src = (task.from_x, task.from_y)
        dst = (task.to_x, task.to_y)

        # Graveyard slots before the destination slot are already filled.
        m = GRAVEYARD_RE.match(task.to_label)
        if m:
            color, slot = m.group(1), int(m.group(2))
            for i in range(slot):
                graveyard_used.setdefault("%s-%d" % (color, i),
                                          self.geometry.graveyard_slot(color, i))

        # ---- pick
        self.approach(index, src, self._obstacle_points(occupied, graveyard_used),
                      task.from_label)
        if self.verify:
            self.feedback(index, "verify", "looking for %s %s" % (
                task.piece_color, task.piece_type))
            try:
                rospy.wait_for_service("verify_piece", timeout=5.0)
                res = self.verify_srv(task.piece_color, task.piece_type)
                if res.matches:
                    self.feedback(index, "verify", "OK: %s" % res.message)
                else:
                    text = "MISMATCH: %s" % res.message
                    self.feedback(index, "verify", text)
                    if self.abort_on_mismatch:
                        raise MoveAborted(text)
            except rospy.ROSException:
                rospy.logwarn("[%s] verify_piece not available - skipping", self.robot_name)

        self.feedback(index, "pick", task.model_name)
        res = self.attach(self.robot_name, task.model_name)
        if not res.success:
            raise MoveAborted("pick failed: %s" % res.message)
        occupied.pop(task.from_label, None)

        # ---- place
        self.approach(index, dst, self._obstacle_points(occupied, graveyard_used),
                      task.to_label)
        self.feedback(index, "place", "%s -> %s" % (task.model_name, task.to_label))
        res = self.release(self.robot_name, task.model_name, dst[0], dst[1])
        if not res.success:
            raise MoveAborted("place failed: %s" % res.message)
        if m:
            graveyard_used["%s-%d" % (m.group(1), int(m.group(2)))] = dst
        else:
            occupied[task.to_label] = dst

        if task.promote_to:
            self.feedback(index, "place", "promotion to %s" % task.promote_to)
            res = self.replace(task.model_name, task.promoted_model_name,
                               task.piece_color, task.promote_to, dst[0], dst[1])
            if not res.success:
                raise MoveAborted("promotion failed: %s" % res.message)

        # back off a little so the next approach starts clear of the piece
        self.backoff()

    def backoff(self, distance=0.12, speed=0.1):
        rate = rospy.Rate(20)
        end = rospy.Time.now() + rospy.Duration(distance / speed)
        cmd = Twist()
        cmd.linear.x = -speed
        while rospy.Time.now() < end and not rospy.is_shutdown():
            self.cmd_pub.publish(cmd)
            rate.sleep()
        self.stop()


if __name__ == "__main__":
    rospy.init_node("piece_mover")
    PieceMover()
    rospy.spin()
