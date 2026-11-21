#!/usr/bin/env python3
"""Terminal view of my robot's game: board from my side, status, engine.

    rosrun robot_chess_game chess_monitor.py
"""
import chess
import rospy
from std_msgs.msg import String

from robot_chess_msgs.msg import GameState


def render(board, color):
    ranks = range(7, -1, -1) if color == "white" else range(8)
    files = range(8) if color == "white" else range(7, -1, -1)
    lines = ["    " + "  ".join("abcdefgh"[f] for f in files)]
    for r in ranks:
        row = []
        for f in files:
            piece = board.piece_at(chess.square(f, r))
            row.append(piece.symbol() if piece else ("." if (f + r) % 2 else ":"))
        lines.append("%d | %s |" % (r + 1, "  ".join(row)))
    return "\n".join(lines)


class Monitor(object):

    def __init__(self):
        self.last = None
        rospy.Subscriber("/chess/state", GameState, self.on_state, queue_size=1)
        rospy.Subscriber("/chess/events", String, lambda m: print("  * " + m.data), queue_size=50)

    def on_state(self, st):
        key = (st.fen, st.status, st.status_text)
        if key == self.last:
            return
        self.last = key
        print("")
        print(render(chess.Board(st.fen), st.my_color or "white"))
        hist = st.move_history_san
        moves = " ".join("%d. %s" % (i // 2 + 1, " ".join(hist[i:i + 2]))
                         for i in range(0, len(hist), 2))
        print("moves : %s" % (moves or "-"))
        print("status: %s%s" % (st.status_text, "  (CHECK)" if st.in_check else ""))
        if st.engine:
            print("engine: %s  %+d cp  depth %d  line %s" % (
                st.engine, st.score_cp, st.depth, " ".join(st.principal_variation)))


if __name__ == "__main__":
    rospy.init_node("chess_monitor", anonymous=True)
    Monitor()
    rospy.spin()
