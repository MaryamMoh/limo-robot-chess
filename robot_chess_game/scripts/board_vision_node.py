#!/usr/bin/env python3
"""Reads the board with the camera.

Service   observe_board  robot_chess_msgs/ObserveBoard
Topics    image, camera_info (remap to the camera)  ->  ~debug_image

On a request the node waits for `~frames` fresh images, looks up where the
camera is relative to the robot (TF base_footprint -> camera optical frame;
for the wrist camera this follows the arm's joint angles), computes where
the board is in the camera from the configured board pose, averages the
per-square colours of the frames and classifies them
(robot_chess.board_vision).  With calibrate=true the colours of the start
position are learnt first.
"""
import threading

import chess
import numpy as np
import rospy
import tf2_ros
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image

from robot_chess.board_geometry import BoardGeometry
from robot_chess.board_vision import BoardVision
from robot_chess.move_inference import occupancy
from robot_chess_msgs.srv import ObserveBoard, ObserveBoardResponse


def transform_matrix(t):
    q, p = t.transform.rotation, t.transform.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    m = np.eye(4)
    m[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    m[:3, 3] = [p.x, p.y, p.z]
    return m


class BoardVisionNode(object):

    def __init__(self):
        self.g = BoardGeometry.from_dict(rospy.get_param("~board", {}))
        self.vision = BoardVision(self.g, sample_height=rospy.get_param("~sample_height", 0.012))
        self.frames = rospy.get_param("~frames", 3)
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")
        self.settle = rospy.get_param("~settle_time", 0.5)
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.image, self.seq, self.K = None, 0, None
        self.tf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf)
        self.debug_pub = rospy.Publisher("~debug_image", Image, queue_size=1)
        rospy.Subscriber("image", Image, self.on_image, queue_size=1, buff_size=2 ** 24)
        rospy.Subscriber("camera_info", CameraInfo, self.on_info, queue_size=1)
        rospy.Service("observe_board", ObserveBoard, self.on_observe)

    def on_image(self, msg):
        with self.lock:
            self.image = msg
            self.seq += 1

    def on_info(self, msg):
        self.K = np.array(msg.K).reshape(3, 3)

    def _fresh_images(self, count, timeout=15.0):
        rospy.sleep(self.settle)                    # let the arm stop shaking
        with self.lock:
            seq = self.seq
        images, deadline = [], rospy.Time.now() + rospy.Duration(timeout)
        while len(images) < count and rospy.Time.now() < deadline:
            with self.lock:
                if self.seq > seq:
                    seq = self.seq
                    images.append(self.image)
            rospy.sleep(0.02)
        return images

    def on_observe(self, req):
        images = self._fresh_images(self.frames)
        if not images or self.K is None:
            return ObserveBoardResponse(success=False, message="no camera images")
        feats, last = [], None
        try:
            for msg in images:
                t = self.tf.lookup_transform(self.base_frame, msg.header.frame_id,
                                             msg.header.stamp, rospy.Duration(1.0))
                cam_from_board = np.linalg.inv(transform_matrix(t)) @ \
                    self.g.board_to_robot_matrix()
                bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                feats.append(self.vision.features(bgr, cam_from_board, self.K))
                last = (bgr, cam_from_board, msg.header)
        except (tf2_ros.TransformException, ValueError) as exc:
            return ObserveBoardResponse(success=False, message=str(exc))
        mean = np.mean(feats, axis=0)

        if req.calibrate:
            self.vision.calibrate(mean, occupancy(chess.Board()))
            rospy.loginfo("board vision calibrated on the start position")
        try:
            occ, conf = self.vision.classify(mean)
        except RuntimeError as exc:
            return ObserveBoardResponse(success=False, message=str(exc))

        if self.debug_pub.get_num_connections():
            out = self.bridge.cv2_to_imgmsg(self.vision.draw(last[0], last[1], self.K, occ), "bgr8")
            out.header = last[2]
            self.debug_pub.publish(out)
        return ObserveBoardResponse(success=True, occupancy=occ, confidence=conf,
                                    message="min confidence %.2f" % min(conf))


if __name__ == "__main__":
    rospy.init_node("board_vision")
    BoardVisionNode()
    rospy.spin()
