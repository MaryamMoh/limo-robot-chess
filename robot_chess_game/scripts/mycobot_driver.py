#!/usr/bin/env python3
"""Driver for the real myCobot 280 (pymycobot), with the same interface as
the simulated ros_control controllers:

    arm_controller/follow_joint_trajectory      control_msgs/FollowJointTrajectory
    gripper_controller/follow_joint_trajectory  (finger opening -> gripper value)
    joint_states                                 sensor_msgs/JointState (10 Hz)

so arm_executor.py runs unchanged on the real robot.  Trajectory points are
streamed with send_radians() at their time stamps ("fresh mode": the newest
command replaces the previous one); the goal succeeds when the measured
angles are within `goal_tolerance` of the last point.

Parameters: ~port (/dev/ttyTHS1 on the LIMO's Jetson), ~baud (1000000),
~speed (0..100), ~goal_tolerance [rad], ~gripper_open_value / ~gripper_closed_value
"""
import math
import threading

import actionlib
import rospy
from control_msgs.msg import (FollowJointTrajectoryAction,
                              FollowJointTrajectoryResult)
from sensor_msgs.msg import JointState

from robot_chess.kinematics import JOINT_NAMES

FINGERS = ["finger_left_joint", "finger_right_joint"]
FINGER_OPEN = 0.012     # finger travel that corresponds to "fully open" [m]

try:
    from pymycobot import MyCobot280 as MyCobot
except ImportError:                     # older pymycobot versions
    from pymycobot.mycobot import MyCobot


class MyCobotDriver(object):

    def __init__(self):
        port = rospy.get_param("~port", "/dev/ttyTHS1")
        baud = rospy.get_param("~baud", 1000000)
        self.speed = rospy.get_param("~speed", 40)
        self.tolerance = rospy.get_param("~goal_tolerance", 0.03)
        self.open_value = rospy.get_param("~gripper_open_value", 100)
        self.closed_value = rospy.get_param("~gripper_closed_value", 0)
        self.lock = threading.Lock()
        self.mc = MyCobot(port, baud)
        rospy.sleep(0.1)
        self.mc.set_fresh_mode(1)
        self.q = [0.0] * 6
        self.finger = FINGER_OPEN

        self.js_pub = rospy.Publisher("joint_states", JointState, queue_size=5)
        rospy.Timer(rospy.Duration(0.1), self.publish_joints)
        self.arm_server = actionlib.SimpleActionServer(
            "arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction,
            execute_cb=self.execute_arm, auto_start=False)
        self.gripper_server = actionlib.SimpleActionServer(
            "gripper_controller/follow_joint_trajectory", FollowJointTrajectoryAction,
            execute_cb=self.execute_gripper, auto_start=False)
        self.arm_server.start()
        self.gripper_server.start()
        rospy.loginfo("myCobot driver ready on %s", port)

    def read_angles(self):
        with self.lock:
            q = self.mc.get_radians()
        if q and len(q) == 6:
            self.q = list(q)
        return self.q

    def publish_joints(self, _event):
        q = self.read_angles()
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = JOINT_NAMES + FINGERS
        msg.position = list(q) + [self.finger, self.finger]
        self.js_pub.publish(msg)

    def execute_arm(self, goal):
        traj = goal.trajectory
        index = [traj.joint_names.index(n) for n in JOINT_NAMES]
        start = rospy.Time.now()
        for point in traj.points:
            target = [point.positions[i] for i in index]
            wait = (start + point.time_from_start - rospy.Time.now()).to_sec()
            if wait > 0:
                rospy.sleep(wait)
            if self.arm_server.is_preempt_requested():
                self.arm_server.set_preempted()
                return
            with self.lock:
                self.mc.send_radians(target, self.speed)
        final = [traj.points[-1].positions[i] for i in index] if traj.points else self.q
        deadline = rospy.Time.now() + rospy.Duration(15.0)
        while rospy.Time.now() < deadline and not rospy.is_shutdown():
            q = self.read_angles()
            if max(abs(a - b) for a, b in zip(q, final)) < self.tolerance:
                self.arm_server.set_succeeded(FollowJointTrajectoryResult(error_code=0))
                return
            rospy.sleep(0.1)
        self.arm_server.set_aborted(FollowJointTrajectoryResult(
            error_code=FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED,
            error_string="arm did not reach the goal"))

    def execute_gripper(self, goal):
        opening = goal.trajectory.points[-1].positions[0]
        fraction = max(0.0, min(1.0, opening / FINGER_OPEN))
        value = int(round(self.closed_value + fraction * (self.open_value - self.closed_value)))
        with self.lock:
            self.mc.set_gripper_value(value, 50)
        rospy.sleep(1.0)
        self.finger = opening
        self.gripper_server.set_succeeded(FollowJointTrajectoryResult(error_code=0))


if __name__ == "__main__":
    rospy.init_node("mycobot_driver")
    MyCobotDriver()
    rospy.spin()
