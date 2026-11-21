#!/usr/bin/env python3
"""Terminal for one human player.

    rosrun robot_chess_game player_console.py white
    rosrun robot_chess_game player_console.py black

Each player runs a console (on the same computer or on any machine
connected to the same ROS master).  The board is drawn from the player's
side, moves are typed in SAN (Nf3, exd5, O-O, e8=Q) or UCI (g1f3, e7e8q).
"""
import sys
import threading

import chess
import rospy
from std_msgs.msg import String

from robot_chess_msgs.msg import GameState
from robot_chess_msgs.srv import SubmitMove

HELP = """commands:
  <move>     play a move, e.g.  e4  Nf3  exd5  O-O  e8=Q  or  e2e4
  moves      list your legal moves
  board      show the board
  history    show the move list
  draw       offer / accept / claim a draw
  resign     resign the game
  help       this text
  quit       leave the console (the game goes on)"""


def render(board, color):
    ranks = range(7, -1, -1) if color == "white" else range(8)
    files = range(8) if color == "white" else range(7, -1, -1)
    lines = ["    " + "  ".join("abcdefgh"[f] for f in files)]
    lines.append("  +" + "-" * 25 + "+")
    for r in ranks:
        row = []
        for f in files:
            piece = board.piece_at(chess.square(f, r))
            row.append(piece.symbol() if piece else ("." if (f + r) % 2 else ":"))
        lines.append("%d | %s |" % (r + 1, "  ".join(row)))
    lines.append("  +" + "-" * 25 + "+")
    lines.append("    upper case = white, lower case = black")
    return "\n".join(lines)


class PlayerConsole(object):

    def __init__(self, color):
        self.color = color
        self.state = None
        self.lock = threading.Lock()
        rospy.Subscriber("/chess/state", GameState, self.on_state, queue_size=1)
        rospy.Subscriber("/chess/events", String, self.on_event, queue_size=20)
        self.submit = rospy.ServiceProxy("/chess/submit_move", SubmitMove)

    def board(self):
        with self.lock:
            return chess.Board(self.state.fen) if self.state else chess.Board()

    def on_event(self, msg):
        print("\n  * %s" % msg.data)

    def on_state(self, msg):
        with self.lock:
            previous = self.state
            self.state = msg
        changed = previous is None or previous.fen != msg.fen or previous.status != msg.status
        if changed:
            self.show()

    def show(self):
        st = self.state
        if st is None:
            print("waiting for the referee ...")
            return
        print("")
        print(render(chess.Board(st.fen), self.color))
        if st.last_move_san:
            print("last move: %s" % st.last_move_san)
        print("status:    %s" % st.status_text)
        if st.status == GameState.STATUS_WAITING_FOR_MOVE and st.turn == self.color:
            print(">>> YOUR MOVE (%s)" % self.color)
        sys.stdout.write("%s> " % self.color)
        sys.stdout.flush()

    def handle(self, line):
        cmd = line.strip()
        if not cmd:
            return True
        low = cmd.lower()
        if low in ("quit", "exit"):
            return False
        if low == "help":
            print(HELP)
        elif low == "board":
            self.show()
        elif low == "history":
            hist = self.state.move_history_san if self.state else []
            pairs = ["%d. %s" % (i // 2 + 1, " ".join(hist[i:i + 2]))
                     for i in range(0, len(hist), 2)]
            print(" ".join(pairs) or "(no moves yet)")
        elif low == "moves":
            b = self.board()
            print(" ".join(sorted(b.san(m) for m in b.legal_moves)))
        else:
            try:
                rospy.wait_for_service("/chess/submit_move", timeout=5.0)
                res = self.submit(self.color, cmd)
                print(("OK: " if res.accepted else "REJECTED: ") + res.message)
            except (rospy.ServiceException, rospy.ROSException) as exc:
                print("referee not reachable: %s" % exc)
        return True

    def run(self):
        print("Robot chess - you play %s. Type 'help' for commands." % self.color.upper())
        rospy.sleep(0.5)
        self.show()
        while not rospy.is_shutdown():
            try:
                line = input()
            except (EOFError, KeyboardInterrupt):
                break
            if not self.handle(line):
                break
            sys.stdout.write("%s> " % self.color)
            sys.stdout.flush()


def main():
    rospy.init_node("player_console", anonymous=True, disable_signals=True)
    args = rospy.myargv(argv=sys.argv)[1:]
    color = (args[0] if args else rospy.get_param("~color", "white")).lower()
    if color not in ("white", "black"):
        print("usage: player_console.py white|black")
        sys.exit(1)
    PlayerConsole(color).run()
    rospy.signal_shutdown("console closed")


if __name__ == "__main__":
    main()
