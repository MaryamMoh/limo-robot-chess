#!/usr/bin/env python3
"""End-to-end check of the simulation (used by CI).

My robot (white) plays against the opponent stand-in (black) for a number
of half-moves.  The test passes when
  * my player recognised every opponent move and played its own moves
    without errors,
  * my player's idea of the game and the opponent's agree move by move,
  * the pieces in Gazebo (ground truth) match my player's position.

    roslaunch robot_chess_bringup simulation.launch gui:=false rviz:=false
    rosrun robot_chess_bringup smoke_test.py _plies:=6
"""
import sys

import chess
import rospy
from std_msgs.msg import String

from robot_chess.move_inference import differences, occupancy
from robot_chess_msgs.msg import GameState


class SmokeTest(object):

    def __init__(self):
        self.plies = rospy.get_param("~plies", 6)
        self.timeout = rospy.get_param("~timeout", 3600.0)
        self.state = None
        self.truth = None
        self.errors = []
        self.opponent_log = []
        rospy.Subscriber("/chess/state", GameState, self.on_state, queue_size=10)
        rospy.Subscriber("/chess/events", String, self.on_event, queue_size=50)
        rospy.Subscriber("/chess_sim/occupancy", String, self.on_truth, queue_size=1)
        rospy.Subscriber("/chess_sim/opponent", String, self.on_opponent, queue_size=20)

    def on_state(self, msg):
        self.state = msg
        if msg.status == GameState.STATUS_ERROR:
            self.errors.append(msg.status_text)

    def on_event(self, msg):
        print("[player] %s" % msg.data)
        sys.stdout.flush()

    def on_opponent(self, msg):
        print("[opponent] %s" % msg.data)
        self.opponent_log.append(msg.data)
        sys.stdout.flush()

    def on_truth(self, msg):
        self.truth = msg.data

    def run(self):
        deadline = rospy.Time.now() + rospy.Duration(self.timeout)
        while not rospy.is_shutdown():
            st = self.state
            if st is not None and (
                    st.status == GameState.STATUS_GAME_OVER or
                    (len(st.move_history_san) >= self.plies and
                     st.status == GameState.STATUS_WAITING)):
                break
            if rospy.Time.now() > deadline:
                raise RuntimeError("timeout after %d half-moves (status: %s)" % (
                    len(st.move_history_san) if st else 0, st.status_text if st else "none"))
            rospy.sleep(1.0)
        rospy.sleep(3.0)
        st = self.state
        print("game: %s" % " ".join(st.move_history_san))
        print("position: %s" % st.fen)
        if self.errors:
            raise RuntimeError("player reported errors: %s" % self.errors)

        # both robots saw the same game
        played = [line.split()[-1] for line in self.opponent_log if line.startswith("played")]
        seen = [line.split()[-1] for line in self.opponent_log if line.startswith("saw")]
        mine, theirs = st.move_history_san[0::2], st.move_history_san[1::2]
        print("opponent played %s / my player saw %s" % (played, theirs))
        print("my player played %s / opponent saw %s" % (mine, seen))
        if played[:len(theirs)] != theirs or mine[:len(seen)] != seen:
            raise RuntimeError("the two robots disagree about the game")

        # the real board matches the player's position
        board = chess.Board(st.fen)
        wrong = differences(self.truth, occupancy(board))
        print("ground truth %s\nplayer       %s" % (self.truth, occupancy(board)))
        if wrong:
            raise RuntimeError("Gazebo board differs on %s" % [chess.square_name(s) for s in wrong])
        print("SMOKE TEST PASSED (%d half-moves)" % len(st.move_history_san))


if __name__ == "__main__":
    rospy.init_node("smoke_test")
    try:
        SmokeTest().run()
    except Exception as exc:
        print("SMOKE TEST FAILED: %s" % exc)
        sys.exit(1)
