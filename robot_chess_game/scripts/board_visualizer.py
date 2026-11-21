#!/usr/bin/env python3
"""RViz view of the logical game: board, pieces, coordinates and status.

Subscribes  /chess/state           robot_chess_msgs/GameState
Publishes   /chess/board_markers   visualization_msgs/MarkerArray (frame: map)
"""
import chess
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from robot_chess.board_geometry import BoardGeometry
from robot_chess.pieces import BAND_RGB, BODY_RGB, PIECE_HEIGHT, PIECE_TYPE_NAME
from robot_chess_msgs.msg import GameState

LETTER = {"pawn": "P", "knight": "N", "bishop": "B", "rook": "R", "queen": "Q", "king": "K"}


def rgba(rgb, a=1.0):
    return ColorRGBA(rgb[0], rgb[1], rgb[2], a)


class BoardVisualizer(object):

    def __init__(self):
        self.g = BoardGeometry.from_dict(rospy.get_param("~board", {}))
        self.frame = rospy.get_param("~frame", "map")
        self.pub = rospy.Publisher("/chess/board_markers", MarkerArray, queue_size=1,
                                   latch=True)
        self.state = None
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

    def board_markers(self):
        out = []
        s = self.g.square_size
        for light in (True, False):
            m = self.marker("squares", int(light), Marker.CUBE_LIST)
            m.scale.x = m.scale.y = s
            m.scale.z = 0.01
            m.color = rgba((0.86, 0.85, 0.80) if light else (0.30, 0.30, 0.33), 0.8)
            for sq in chess.SQUARES:
                if ((chess.square_file(sq) + chess.square_rank(sq)) % 2 == 1) == light:
                    x, y = self.g.square_center(sq)
                    m.points.append(Point(x, y, -0.005))
            out.append(m)
        mid = 0
        for i in range(8):
            for text, bx, by in (("abcdefgh"[i], (i - 3.5) * s, -4.35 * s),
                                 (str(i + 1), -4.35 * s, (i - 3.5) * s)):
                m = self.marker("labels", mid, Marker.TEXT_VIEW_FACING)
                m.pose.position.x, m.pose.position.y = self.g.board_to_map(bx, by)
                m.pose.position.z = 0.05
                m.scale.z = 0.25
                m.color = rgba((1.0, 1.0, 1.0))
                m.text = text
                out.append(m)
                mid += 1
        return out

    def piece_markers(self, board):
        out = []
        for i, (sq, piece) in enumerate(sorted(board.piece_map().items())):
            ptype = PIECE_TYPE_NAME[piece.piece_type]
            color = "white" if piece.color == chess.WHITE else "black"
            x, y = self.g.square_center(sq)
            h = PIECE_HEIGHT[ptype]
            body = self.marker("pieces", i, Marker.CYLINDER)
            body.pose.position.x, body.pose.position.y, body.pose.position.z = x, y, h / 2
            body.scale.x = body.scale.y = 0.18
            body.scale.z = h
            body.color = rgba(BODY_RGB[color])
            band = self.marker("bands", i, Marker.CYLINDER)
            band.pose.position.x, band.pose.position.y, band.pose.position.z = x, y, 0.18
            band.scale.x = band.scale.y = 0.19
            band.scale.z = 0.12
            band.color = rgba(BAND_RGB[ptype])
            label = self.marker("letters", i, Marker.TEXT_VIEW_FACING)
            label.pose.position.x, label.pose.position.y = x, y
            label.pose.position.z = h + 0.15
            label.scale.z = 0.22
            label.color = rgba((1.0, 0.9, 0.2))
            label.text = LETTER[ptype] if color == "white" else LETTER[ptype].lower()
            out += [body, band, label]
        return out

    def publish(self):
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        arr.markers += self.board_markers()
        board = chess.Board(self.state.fen) if self.state else chess.Board()
        arr.markers += self.piece_markers(board)
        status = self.marker("status", 0, Marker.TEXT_VIEW_FACING)
        status.pose.position.x, status.pose.position.y = self.g.board_to_map(0.0, 0.0)
        status.pose.position.z = 1.5
        status.scale.z = 0.35
        status.color = rgba((1.0, 1.0, 1.0))
        status.text = self.state.status_text if self.state else "waiting for referee"
        arr.markers.append(status)
        self.pub.publish(arr)


if __name__ == "__main__":
    rospy.init_node("board_visualizer")
    BoardVisualizer()
    rospy.spin()
