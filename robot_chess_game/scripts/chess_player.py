#!/usr/bin/env python3
"""The chess player: my robot's brain.

It keeps the state of the game (python-chess: position, legal moves, check,
mate, draws), watches the real board through the camera, and plays its own
moves with the arm.  There is no shared referee: the opponent is another
robot with its own software, and the only thing both share is the board.

    start:  arm to the observe pose, learn the board colours from the start
            position (board_vision, calibrate=true)
    loop:
      my turn        engine picks the best move -> tasks (move_planner)
                     -> arm (execute_move) -> camera checks the result
      opponent turn  read the board every ~observe_period seconds; when it
                     differs from the current position and the same reading
                     came ~stable_observations times in a row (the other arm
                     has finished), find the legal move that explains it
                     (move_inference); unexplained changes are reported and
                     waited out

Publishes /chess/state (robot_chess_msgs/GameState) and /chess/events.
Services  ~new_game (std_srvs/Trigger)
          ~override_opponent_move (robot_chess_msgs/OverrideMove) - when the
          camera cannot tell (e.g. an under-promotion)
"""
import threading

import actionlib
import chess
import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

from robot_chess.engine import make_engine
from robot_chess.move_inference import differences, infer_move, occupancy
from robot_chess.move_planner import parse_move, plan_move
from robot_chess.pieces import chess_color, color_name
from robot_chess_msgs.msg import ExecuteMoveAction, ExecuteMoveGoal, GameState, PieceTask
from robot_chess_msgs.srv import ObserveBoard, OverrideMove, OverrideMoveResponse


class ChessPlayer(object):

    def __init__(self):
        cfg = rospy.get_param("~player", {})
        self.my_color = cfg.get("color", "white")
        self.me = chess_color(self.my_color)
        self.engine = make_engine(cfg.get("engine", "internal"),
                                  time_limit=cfg.get("think_time", 3.0),
                                  max_depth=cfg.get("max_depth", 5),
                                  skill_level=cfg.get("skill_level"))
        self.stable_needed = int(cfg.get("stable_observations", 3))
        self.period = float(cfg.get("observe_period", 1.0))
        self.verify_retries = int(cfg.get("verify_retries", 3))

        self.lock = threading.RLock()
        self.state_pub = rospy.Publisher("/chess/state", GameState, queue_size=1, latch=True)
        self.event_pub = rospy.Publisher("/chess/events", String, queue_size=20)
        self.observe = rospy.ServiceProxy("observe_board", ObserveBoard)
        self.observe_pose = rospy.ServiceProxy("arm_executor/observe_pose", Trigger)
        self.arm = actionlib.SimpleActionClient("execute_move", ExecuteMoveAction)
        self.reset_game()

        rospy.Service("~new_game", Trigger, self.on_new_game)
        rospy.Service("~override_opponent_move", OverrideMove, self.on_override)

    # ---------------------------------------------------------------- state
    def reset_game(self):
        with self.lock:
            self.board = chess.Board()
            self.graveyard_used = 0
            self.history = []
            self.status = GameState.STATUS_CALIBRATING
            self.status_text = "starting"
            self.observed = ""
            self.analysis = None
            self.analysis_pv = []
            self.calibrated = False
            self.publish()

    def say(self, text):
        rospy.loginfo(text)
        self.event_pub.publish(String(text))

    def set_status(self, status, text):
        if (status, text) != (self.status, self.status_text):
            self.say(text)
        self.status, self.status_text = status, text
        self.publish()

    def publish(self):
        msg = GameState()
        msg.stamp = rospy.Time.now()
        msg.my_color = self.my_color
        msg.fen = self.board.fen()
        msg.turn = color_name(self.board.turn)
        msg.status = self.status
        msg.status_text = self.status_text
        msg.last_move_san = self.history[-1] if self.history else ""
        msg.move_history_san = list(self.history)
        msg.result = self.board.result(claim_draw=False) if self.board.is_game_over() else "*"
        msg.in_check = self.board.is_check()
        msg.observed_occupancy = self.observed
        if self.analysis is not None:
            msg.engine = self.analysis.engine
            msg.score_cp = int(self.analysis.score)
            msg.depth = int(self.analysis.depth)
            msg.principal_variation = self.analysis_pv
        self.state_pub.publish(msg)

    def push(self, move, who):
        san = self.board.san(move)
        self.board.push(move)
        self.history.append(san)
        self.say("%s played %s" % (who, san))

    # ------------------------------------------------------------ helpers
    def read_board(self, calibrate=False):
        rospy.wait_for_service("observe_board")
        res = self.observe(calibrate)
        if not res.success:
            raise RuntimeError(res.message)
        self.observed = res.occupancy
        return res.occupancy

    def wait_for_services(self):
        self.set_status(GameState.STATUS_CALIBRATING, "waiting for arm and camera")
        rospy.wait_for_service("arm_executor/observe_pose")
        self.arm.wait_for_server()

    def calibrate(self):
        """Learn the board colours from the start position.  Accepted when a
        calibration and a second, independent reading both give the start
        position."""
        res = self.observe_pose()
        if not res.success:
            raise RuntimeError(res.message)
        rospy.sleep(1.0)                          # let the arm settle
        start = occupancy(chess.Board())
        for calibrate in (True, False):
            occ = self.read_board(calibrate=calibrate)
            wrong = differences(occ, start)
            if wrong:
                raise RuntimeError("squares %s do not look like the start position"
                                   % [chess.square_name(s) for s in wrong])
        self.calibrated = True

    # ------------------------------------------------------------- my move
    def play_my_move(self):
        self.set_status(GameState.STATUS_THINKING,
                        "thinking (%s engine)" % self.engine.name)
        before = self.board.copy()
        result = self.engine.choose(before)
        move = result.move
        self.analysis = result
        self.analysis_pv = self._pv_san(before, result.pv)
        san = before.san(move)
        self.say("best move %s (score %+d cp, depth %d, line %s)" % (
            san, result.score, result.depth, " ".join(self.analysis_pv)))

        tasks, graveyard_used = plan_move(before, move, self.graveyard_used)
        goal = ExecuteMoveGoal(move_uci=move.uci(),
                               tasks=[PieceTask(**t.as_dict()) for t in tasks])
        self.set_status(GameState.STATUS_MOVING, "arm is playing %s" % san)
        self.arm.send_goal(goal, feedback_cb=lambda fb: self.say(
            "[arm] %s: %s" % (fb.stage, fb.text)))
        self.arm.wait_for_result()
        res = self.arm.get_result()
        if res is None or not res.success:
            raise RuntimeError("arm failed to play %s: %s" % (
                san, res.message if res else "no result"))

        after = before.copy()
        after.push(move)
        expected = occupancy(after)
        for _ in range(self.verify_retries):
            occ = self.read_board()
            wrong = differences(occ, expected)
            if not wrong:
                break
            rospy.sleep(0.5)
        else:
            self.say("WARNING: after %s the board looks different on %s" % (
                san, [chess.square_name(s) for s in wrong]))
        self.graveyard_used = graveyard_used
        self.push(move, "I (%s)" % self.my_color)

    @staticmethod
    def _pv_san(board, pv):
        out, b = [], board.copy()
        for m in pv:
            if m not in b.legal_moves:
                break
            out.append(b.san(m))
            b.push(m)
        return out

    # ------------------------------------------------------- their move
    def wait_for_opponent(self):
        self.set_status(GameState.STATUS_WAITING, "waiting for the opponent")
        current = occupancy(self.board)
        last, count = None, 0
        while not rospy.is_shutdown():
            with self.lock:
                if self.board.turn == self.me:          # set by an override
                    return
            occ = self.read_board()
            self.publish()
            if occ == current:
                last, count = None, 0
            else:
                count = count + 1 if occ == last else 1
                last = occ
                if count >= self.stable_needed:
                    with self.lock:
                        result = infer_move(self.board, occ)
                        if result.status == "move":
                            self.push(result.move, "opponent")
                            return
                    changed = [chess.square_name(s) for s in result.changed]
                    self.set_status(GameState.STATUS_UNCLEAR,
                                    "board changed on %s but no legal move explains it - "
                                    "waiting (or use override_opponent_move)" % changed)
            rospy.sleep(self.period)

    # ------------------------------------------------------------ services
    def on_override(self, req):
        with self.lock:
            if self.board.turn == self.me:
                return OverrideMoveResponse(False, "it is my turn")
            try:
                move = parse_move(self.board, req.move)
            except ValueError as exc:
                return OverrideMoveResponse(False, str(exc))
            self.push(move, "opponent (override)")
            self.publish()
        return OverrideMoveResponse(True, "accepted")

    def on_new_game(self, _req):
        self.reset_game()
        self.say("new game - put the pieces in the start position")
        return TriggerResponse(True, "new game")

    # ------------------------------------------------------------- main
    def run(self):
        self.wait_for_services()
        while not rospy.is_shutdown():
            if not self.calibrated:
                try:
                    self.calibrate()
                    self.say("ready - I play %s" % self.my_color)
                except (RuntimeError, rospy.ServiceException) as exc:
                    self.set_status(GameState.STATUS_CALIBRATING,
                                    "calibrating: %s - retrying" % exc)
                    rospy.sleep(2.0)
                continue
            try:
                if self.board.is_game_over(claim_draw=True):
                    self.set_status(GameState.STATUS_GAME_OVER, "game over: %s" %
                                    self.board.result(claim_draw=True))
                    rospy.sleep(1.0)
                    continue
                if self.board.turn == self.me:
                    self.play_my_move()
                else:
                    self.wait_for_opponent()
            except (RuntimeError, rospy.ServiceException) as exc:
                self.set_status(GameState.STATUS_ERROR, "error: %s - retrying" % exc)
                rospy.sleep(3.0)


if __name__ == "__main__":
    rospy.init_node("chess_player")
    ChessPlayer().run()
