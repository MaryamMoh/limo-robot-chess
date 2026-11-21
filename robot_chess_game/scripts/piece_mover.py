#!/usr/bin/env python3
"""Executes chess moves physically with one LIMO PRO.

Action server  execute_move  (robot_chess_msgs/ExecuteMove)

For every PieceTask of the goal:
    1. approach  - plan the shortest route on the road network
                   (robot_chess.road_network) to one of the approach points
                   of the piece and drive it; turn to face the piece
    2. verify    - ask the camera (verify_piece) whether the expected piece is there
    3. pick      - carrier attach (simulation: piece goes on the robot plate,
                   real robot: an assistant is asked to load it)
    4. transport - drive to an approach point of the destination
    5. place     - carrier release (+ replace for a pawn promotion)
After the last task the robot drives back to its home pose.

Driving: the route is a list of straight, axis-parallel segments.  On each
waypoint the robot turns in place towards the next one and then drives
straight with a proportional heading controller.  The pose comes from TF
(map -> base, i.e. AMCL + EKF).  If the LiDAR sees something closer than
~safety_distance in front, the robot stops and waits until the way is free.

Parameters
    ~robot_name        name used by the carrier (Gazebo model name)
    ~color             "white" | "black"  (selects the home pose)
    ~board, ~home      board geometry / home poses (board yaml)
    ~base_frame        robot base frame (e.g. white/base_footprint)
    ~global_frame      map
    ~carrier_ns        namespace of the carrier services (/chess_carrier)
    ~verify_with_camera, ~abort_on_mismatch
    ~max_speed, ~max_turn_speed, ~safety_distance, ~blocked_timeout
"""
import math
import threading

import actionlib
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan

from robot_chess.board_geometry import BoardGeometry, normalize_angle
from robot_chess.road_network import RoadNetwork, heading_error
from robot_chess_msgs.msg import (ExecuteMoveAction, ExecuteMoveFeedback,
                                  ExecuteMoveResult)
from robot_chess_msgs.srv import (AttachPiece, ReleasePiece, ReplacePiece,
                                  VerifyPiece)


class MoveAborted(Exception):
    pass


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class PieceMover(object):

    def __init__(self):
        ns = rospy.get_namespace().strip("/")
        self.robot_name = rospy.get_param("~robot_name", ns or "limo")
        self.color = rospy.get_param("~color", "white")
        self.geometry = BoardGeometry.from_dict(rospy.get_param("~board", {}))
        self.geometry.validate()
        homes = rospy.get_param("~home", {})
        self.network = RoadNetwork(self.geometry, {
            c: (h["x"], h["y"], h["yaw"]) for c, h in homes.items()})
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")
        self.global_frame = rospy.get_param("~global_frame", "map")
        self.verify = rospy.get_param("~verify_with_camera", True)
        self.abort_on_mismatch = rospy.get_param("~abort_on_mismatch", False)
        self.max_speed = rospy.get_param("~max_speed", 0.3)
        self.max_turn = rospy.get_param("~max_turn_speed", 0.8)
        self.safety_distance = rospy.get_param("~safety_distance", 0.25)
        self.blocked_timeout = rospy.get_param("~blocked_timeout", 60.0)
        self.position_tolerance = rospy.get_param("~position_tolerance", 0.03)
        self.start_tolerance = rospy.get_param("~start_tolerance", 0.15)
        self.heading_tolerance = rospy.get_param("~heading_tolerance", 0.03)
        self.return_home = rospy.get_param("~return_home", True)
        carrier = rospy.get_param("~carrier_ns", "/chess_carrier")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.route_pub = rospy.Publisher("route", Path, queue_size=1, latch=True)
        self.scan_lock = threading.Lock()
        self.front_range = float("inf")
        rospy.Subscriber("scan", LaserScan, self.on_scan, queue_size=1)

        rospy.loginfo("[%s] waiting for carrier services in %s", self.robot_name, carrier)
        for name in ("attach", "release", "replace"):
            rospy.wait_for_service(carrier + "/" + name)
        self.attach = rospy.ServiceProxy(carrier + "/attach", AttachPiece)
        self.release = rospy.ServiceProxy(carrier + "/release", ReleasePiece)
        self.replace = rospy.ServiceProxy(carrier + "/replace", ReplacePiece)
        self.verify_srv = rospy.ServiceProxy("verify_piece", VerifyPiece)

        rospy.loginfo("[%s] waiting for localisation (%s -> %s)", self.robot_name,
                      self.global_frame, self.base_frame)
        while not rospy.is_shutdown():
            try:
                self.robot_pose()
                break
            except tf2_ros.TransformException:
                rospy.sleep(0.5)

        self.server = actionlib.SimpleActionServer(
            "execute_move", ExecuteMoveAction, execute_cb=self.execute, auto_start=False)
        self.server.start()
        rospy.loginfo("[%s] piece mover ready", self.robot_name)

    # ================================================================ sensing
    def on_scan(self, scan):
        """Closest return in a +-20 degree cone in front of the robot."""
        best = float("inf")
        angle = scan.angle_min
        for r in scan.ranges:
            if abs(angle) < 0.35 and scan.range_min < r < scan.range_max:
                best = min(best, r)
            angle += scan.angle_increment
        with self.scan_lock:
            self.front_range = best

    def path_blocked(self):
        with self.scan_lock:
            return self.front_range < self.safety_distance

    def robot_pose(self):
        t = self.tf_buffer.lookup_transform(self.global_frame, self.base_frame,
                                            rospy.Time(0), rospy.Duration(2.0))
        tr = t.transform
        return (tr.translation.x, tr.translation.y, yaw_from_quaternion(tr.rotation))

    # ================================================================ driving
    def feedback(self, index, stage, text):
        rospy.loginfo("[%s] task %d %s: %s", self.robot_name, index, stage, text)
        self.server.publish_feedback(ExecuteMoveFeedback(task_index=index, stage=stage,
                                                         text=text))

    def check_preempt(self):
        if self.server.is_preempt_requested() or rospy.is_shutdown():
            self.stop()
            raise MoveAborted("preempted")

    def stop(self):
        self.cmd_pub.publish(Twist())

    def turn_to(self, target_yaw, timeout=20.0):
        """Rotate in place to an absolute heading (map frame)."""
        rate = rospy.Rate(20)
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        while True:
            self.check_preempt()
            err = normalize_angle(target_yaw - self.robot_pose()[2])
            if abs(err) < self.heading_tolerance:
                break
            if rospy.Time.now() > deadline:
                rospy.logwarn("[%s] turn timeout, heading error %.2f rad", self.robot_name, err)
                break
            cmd = Twist()
            w = clamp(2.0 * err, -self.max_turn, self.max_turn)
            cmd.angular.z = math.copysign(max(abs(w), 0.12), w)
            self.cmd_pub.publish(cmd)
            rate.sleep()
        self.stop()

    def drive_to(self, wx, wy):
        """Drive straight to (wx, wy); the robot already faces it.

        The goal counts as reached when the remaining distance *along the
        segment* is below the tolerance, so a small lateral offset near the
        goal never makes the robot circle around it.  Far from the goal the
        robot steers towards it; in the last 15 cm it keeps the segment
        direction.
        """
        rate = rospy.Rate(20)
        x0, y0, _ = self.robot_pose()
        length = math.hypot(wx - x0, wy - y0)
        if length < 1e-3:
            return
        ux, uy = (wx - x0) / length, (wy - y0) / length
        deadline = rospy.Time.now() + rospy.Duration(20.0 + length / 0.05)
        last_log = rospy.Time.now()
        blocked_since = None
        while True:
            self.check_preempt()
            x, y, yaw = self.robot_pose()
            remaining = (wx - x) * ux + (wy - y) * uy
            dist = math.hypot(wx - x, wy - y)
            if rospy.Time.now() > deadline:
                raise MoveAborted("timeout driving to (%.2f, %.2f), robot at (%.2f, %.2f)"
                                  % (wx, wy, x, y))
            if remaining < self.position_tolerance:
                break
            if dist > 0.15:
                err = heading_error(x, y, yaw, wx, wy)
            else:
                err = normalize_angle(math.atan2(uy, ux) - yaw)
            if (rospy.Time.now() - last_log).to_sec() > 5.0:
                last_log = rospy.Time.now()
                rospy.loginfo("[%s] driving: at (%.2f, %.2f, %.2f) -> (%.2f, %.2f), "
                              "%.2f m left, heading error %.2f", self.robot_name,
                              x, y, yaw, wx, wy, remaining, err)
            if abs(err) > 0.6:
                # overshoot or large drift: stop and re-aim
                self.stop()
                self.turn_to(math.atan2(wy - y, wx - x))
                continue
            if self.path_blocked():
                self.stop()
                if blocked_since is None:
                    blocked_since = rospy.Time.now()
                    rospy.logwarn("[%s] obstacle ahead - waiting", self.robot_name)
                elif (rospy.Time.now() - blocked_since).to_sec() > self.blocked_timeout:
                    raise MoveAborted("path blocked near (%.2f, %.2f)" % (x, y))
                rate.sleep()
                continue
            blocked_since = None
            cmd = Twist()
            cmd.linear.x = clamp(0.8 * remaining, 0.05, self.max_speed) * max(0.0, math.cos(err))
            cmd.angular.z = clamp(2.5 * err, -self.max_turn, self.max_turn)
            self.cmd_pub.publish(cmd)
            rate.sleep()
        self.stop()

    def follow(self, waypoints, final_yaw):
        """Turn-then-drive along the waypoints.  The first waypoint is the
        network node the robot stands on; it is skipped when the robot is
        already within `start_tolerance` of it (localisation noise)."""
        for i, (wx, wy) in enumerate(waypoints):
            x, y, _ = self.robot_pose()
            tolerance = self.start_tolerance if i == 0 else self.position_tolerance
            if math.hypot(wx - x, wy - y) < tolerance:
                continue
            self.turn_to(math.atan2(wy - y, wx - x))
            self.drive_to(wx, wy)
        self.turn_to(final_yaw)

    def go(self, index, label, goals, stage="approach"):
        x, y, _ = self.robot_pose()
        try:
            waypoints, final_yaw, length = self.network.plan((x, y), goals)
        except ValueError as exc:
            raise MoveAborted("no route to %s: %s" % (label, exc))
        self.feedback(index, stage, "%s: %.1f m, %d segment(s)" % (
            label, length, max(1, len(waypoints) - 1)))
        self.publish_route([(x, y)] + list(waypoints))
        self.follow(waypoints, final_yaw)

    def publish_route(self, points):
        path = Path()
        path.header.frame_id = self.global_frame
        path.header.stamp = rospy.Time.now()
        for px, py in points:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x, ps.pose.position.y = px, py
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.route_pub.publish(path)

    # ============================================================== execution
    def execute(self, goal):
        try:
            for index, task in enumerate(goal.tasks):
                self.run_task(index, task)
            if self.return_home:
                node, yaw = self.network.home_board(self.color)
                self.go(len(goal.tasks), "home", [(node, yaw)], stage="return")
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

    def run_task(self, index, task):
        # ---- pick
        self.go(index, task.from_label, self.network.approaches(task.from_label))
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

        # ---- place
        self.go(index, task.to_label, self.network.approaches(task.to_label),
                stage="transport")
        self.feedback(index, "place", "%s -> %s" % (task.model_name, task.to_label))
        res = self.release(self.robot_name, task.model_name, task.to_x, task.to_y)
        if not res.success:
            raise MoveAborted("place failed: %s" % res.message)

        if task.promote_to:
            self.feedback(index, "place", "promotion to %s" % task.promote_to)
            res = self.replace(task.model_name, task.promoted_model_name,
                               task.piece_color, task.promote_to, task.to_x, task.to_y)
            if not res.success:
                raise MoveAborted("promotion failed: %s" % res.message)


if __name__ == "__main__":
    rospy.init_node("piece_mover")
    PieceMover()
    rospy.spin()
