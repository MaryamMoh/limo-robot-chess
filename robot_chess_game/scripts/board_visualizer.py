#!/usr/bin/env python3
"""RViz view of what the player believes: board, pieces, status, engine line.

Subscribes /chess/state; publishes /chess/board_markers (frame base_footprint)
and a static TF base_footprint -> chess_board.
"""
import math

import chess
import rospy
import tf2_ros
from geometry_msgs.msg import Point, TransformStamped
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from robot_chess.board_geometry import BoardGeometry
from robot_chess.pieces import BODY_RGB, PIECE_HEIGHT, PIECE_TYPE_NAME
from robot_chess_msgs.msg import GameState

LETTER = {"pawn": "P", "knight": "N", "bishop": "B", "rook": "R", "queen": "Q", "king": "K"}


def rgba(rgb, a=1.0):
    return ColorRGBA(rgb[0], rgb[1], rgb[2], a)


class BoardVisualizer(object):

    def __init__(self):
        self.g = BoardGeometry.from_dict(rospy.get_param("~board", {}))
        self.frame = "chess_board"
        self.pub = rospy.Publisher("/chess/board_markers", MarkerArray, queue_size=1, latch=True)
        self.state = None
        self.static_tf = tf2_ros.StaticTransformBroadcaster()
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = rospy.get_param("~base_frame", "base_footprint")
        t.child_frame_id = self.frame
        t.transform.translation.x = self.g.board_x
        t.transform.translation.y = self.g.board_y
        t.transform.translation.z = self.g.board_z
        t.transform.rotation.z = math.sin(self.g.board_yaw / 2)
        t.transform.rotation.w = math.cos(self.g.board_yaw / 2)
        self.static_tf.sendTransform(t)
        rospy.Subscriber("/chess/state", GameState, self.on_state, queue_size=1)
        rospy.Timer(rospy.Duration(2.0), lambda _e: self.publish())

    def on_state(self, msg):
        self.state = msg
        self.publish()

    def marker(self, ns, mid, mtype):
        m = Marker()
        m.header.frame_id = self.frame
        m.header.stamp = rospy.Time.now()
        m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def publish(self):
        s = self.g.square_size
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        for light in (True, False):
            m = self.marker("squares", int(light), Marker.CUBE_LIST)
            m.scale.x = m.scale.y = s
            m.scale.z = 0.001
            m.color = rgba((0.80, 0.68, 0.48) if light else (0.42, 0.28, 0.17), 0.9)
            for sq in chess.SQUARES:
                if ((chess.square_file(sq) + chess.square_rank(sq)) % 2 == 1) == light:
                    x, y = self.g.square_center_board(sq)
                    m.points.append(Point(x, y, -0.0005))
            arr.markers.append(m)
        board = chess.Board(self.state.fen) if self.state else chess.Board()
        for i, (sq, piece) in enumerate(sorted(board.piece_map().items())):
            ptype = PIECE_TYPE_NAME[piece.piece_type]
            color = "white" if piece.color else "black"
            x, y = self.g.square_center_board(sq)
            h = PIECE_HEIGHT[ptype]
            body = self.marker("pieces", i, Marker.CYLINDER)
            body.pose.position.x, body.pose.position.y, body.pose.position.z = x, y, h / 2
            body.scale.x = body.scale.y = 0.013
            body.scale.z = h
            body.color = rgba(BODY_RGB[color])
            label = self.marker("letters", i, Marker.TEXT_VIEW_FACING)
            label.pose.position.x, label.pose.position.y = x, y
            label.pose.position.z = h + 0.01
            label.scale.z = 0.01
            label.color = rgba((1.0, 0.85, 0.1))
            label.text = LETTER[ptype] if color == "white" else LETTER[ptype].lower()
            arr.markers += [body, label]
        status = self.marker("status", 0, Marker.TEXT_VIEW_FACING)
        status.pose.position.z = 0.12
        status.scale.z = 0.012
        status.color = rgba((1.0, 1.0, 1.0))
        if self.state:
            line = " ".join(self.state.principal_variation[:5])
            status.text = "%s\n%s %+d cp  %s" % (self.state.status_text, self.state.engine,
                                                 self.state.score_cp, line)
        else:
            status.text = "waiting for chess_player"
        arr.markers.append(status)
        self.pub.publish(arr)


if __name__ == "__main__":
    rospy.init_node("board_visualizer")
    BoardVisualizer()
    rospy.spin()
