#!/usr/bin/env python3
"""Turn Gazebo's perfect odometry into realistic, noisy wheel odometry.

Subscribes  odom_ground_truth (nav_msgs/Odometry, from the drive plugin)
Publishes   odom              (nav_msgs/Odometry, noisy, starts at 0,0,0)

The noise follows the odometry motion model in
robot_chess.odometry_noise.OdometryNoiseModel.  The published odometry has
no TF: the EKF (robot_localization) fuses it with the IMU and broadcasts
odom -> base_footprint.
"""
import math

import rospy
from nav_msgs.msg import Odometry

from robot_chess.odometry_noise import OdometryNoiseModel


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class NoisyOdometry(object):

    def __init__(self):
        self.model = OdometryNoiseModel(
            alpha1=rospy.get_param("~alpha1", 0.002),
            alpha2=rospy.get_param("~alpha2", 0.001),
            alpha3=rospy.get_param("~alpha3", 0.002),
            alpha4=rospy.get_param("~alpha4", 0.001),
            seed=rospy.get_param("~seed", None))
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")
        self.twist_var = rospy.get_param("~twist_variance", [0.002, 0.002, 0.01])
        self.pub = rospy.Publisher("odom", Odometry, queue_size=20)
        self.last_stamp = None
        rospy.Subscriber("odom_ground_truth", Odometry, self.callback, queue_size=50)

    def callback(self, msg):
        p = msg.pose.pose
        true_pose = (p.position.x, p.position.y, yaw_of(p.orientation))
        if self.last_stamp is None:
            self.model.reset(true_pose, (0.0, 0.0, 0.0))
            self.last_stamp = msg.header.stamp
            return
        dt = (msg.header.stamp - self.last_stamp).to_sec()
        if dt <= 0.0:
            return
        self.last_stamp = msg.header.stamp
        (x, y, yaw), (d_trans, d_rot) = self.model.update(true_pose)

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.base_frame
        out.pose.pose.position.x = x
        out.pose.pose.position.y = y
        out.pose.pose.orientation.z = math.sin(yaw / 2.0)
        out.pose.pose.orientation.w = math.cos(yaw / 2.0)
        out.twist.twist.linear.x = d_trans / dt
        out.twist.twist.angular.z = d_rot / dt

        pose_cov = [0.0] * 36
        pose_cov[0] = pose_cov[7] = 0.05
        pose_cov[14] = pose_cov[21] = pose_cov[28] = 1e6
        pose_cov[35] = 0.1
        twist_cov = [0.0] * 36
        twist_cov[0], twist_cov[7], twist_cov[35] = self.twist_var
        twist_cov[14] = twist_cov[21] = twist_cov[28] = 1e6
        out.pose.covariance = pose_cov
        out.twist.covariance = twist_cov
        self.pub.publish(out)


if __name__ == "__main__":
    rospy.init_node("noisy_odometry")
    NoisyOdometry()
    rospy.spin()
