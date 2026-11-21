#!/usr/bin/env python3
"""Camera node: identifies the chess piece in front of the robot.

Subscribes  image                  (sensor_msgs/Image, remap to the colour camera)
Publishes   vision/debug_image     annotated image (view in RViz / rqt_image_view)
Service     verify_piece           robot_chess_msgs/VerifyPiece

On a verify request the node collects `~frames` fresh images, classifies
each of them (robot_chess.vision.detect_piece) and returns the majority
vote, which makes the answer robust against single noisy frames.
"""
import collections
import threading

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

from robot_chess.vision import detect_piece, draw_detection
from robot_chess_msgs.srv import VerifyPiece, VerifyPieceResponse


class PieceVision(object):

    def __init__(self):
        self.frames = rospy.get_param("~frames", 5)
        self.timeout = rospy.get_param("~timeout", 5.0)
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest = None
        self.seq = 0
        self.debug_pub = rospy.Publisher("vision/debug_image", Image, queue_size=1)
        rospy.Subscriber("image", Image, self.on_image, queue_size=1, buff_size=2 ** 24)
        rospy.Service("verify_piece", VerifyPiece, self.on_verify)

    def on_image(self, msg):
        with self.lock:
            self.latest = msg
            self.seq += 1

    def _wait_frame(self, after_seq):
        deadline = rospy.Time.now() + rospy.Duration(self.timeout)
        rate = rospy.Rate(50)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            with self.lock:
                if self.seq > after_seq and self.latest is not None:
                    return self.latest, self.seq
            rate.sleep()
        return None, after_seq

    def on_verify(self, req):
        with self.lock:
            seq = self.seq
        detections = []
        for _ in range(self.frames):
            msg, seq = self._wait_frame(seq)
            if msg is None:
                break
            try:
                bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            except CvBridgeError as exc:
                rospy.logwarn("cv_bridge: %s", exc)
                continue
            det = detect_piece(bgr)
            detections.append(det)
            if self.debug_pub.get_num_connections():
                out = self.bridge.cv2_to_imgmsg(draw_detection(bgr, det), "bgr8")
                out.header = msg.header
                self.debug_pub.publish(out)

        if not detections:
            return VerifyPieceResponse(found=False, matches=False, confidence=0.0,
                                       message="no camera images received")

        found = [d for d in detections if d.found]
        if len(found) * 2 < len(detections):
            return VerifyPieceResponse(found=False, matches=False, confidence=0.0,
                                       message=detections[-1].message)
        votes = collections.Counter((d.color, d.piece_type) for d in found)
        (color, ptype), count = votes.most_common(1)[0]
        confidence = (count / float(len(detections))) * \
            sum(d.confidence for d in found if (d.color, d.piece_type) == (color, ptype)) / count
        matches = (color == req.expected_color and ptype == req.expected_type)
        text = "saw %s %s in %d/%d frames (expected %s %s)" % (
            color, ptype, count, len(detections), req.expected_color, req.expected_type)
        rospy.loginfo("verify_piece: %s", text)
        return VerifyPieceResponse(found=True, matches=matches, detected_color=color,
                                   detected_type=ptype, confidence=confidence,
                                   message=text)


if __name__ == "__main__":
    rospy.init_node("piece_vision")
    PieceVision()
    rospy.spin()
