#!/usr/bin/env python3
"""Chess referee: rules, turn order and dispatching moves to the robots.

Services
    /chess/submit_move  robot_chess_msgs/SubmitMove   (from the player consoles)
    /chess/new_game     std_srvs/Trigger
Topics
    /chess/state        robot_chess_msgs/GameState    (latched)
    /chess/events       std_msgs/String               human readable log

A move is checked with python-chess, decomposed into pick-and-place tasks
(robot_chess.move_planner) and sent to the moving side's robot
(<namespace>/execute_move).  The move is committed to the board only when
the robot reports success; on failure the same player may submit again.

A side without a robot (parameter ~robots) is "virtual": its moves are
applied immediately - used on the real LIMO where the human opponent moves
their own pieces by hand.
"""
import threading

import actionlib
import chess
import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

from robot_chess.board_geometry import BoardGeometry
from robot_chess.move_planner import PhysicalState, parse_move, plan_move
from robot_chess.pieces import initial_square_models, opposite
from robot_chess_msgs.msg import (ExecuteMoveAction, ExecuteMoveGoal, GameState,
                                  PieceTask)
from robot_chess_msgs.srv import SubmitMove, SubmitMoveResponse


class ChessReferee(object):

    def __init__(self):
        self.geometry = BoardGeometry.from_dict(rospy.get_param("~board", {}))
        self.robot_ns = rospy.get_param("~robots", {"white": "/white", "black": "/black"})
        self.lock = threading.RLock()

        self.state_pub = rospy.Publisher("/chess/state", GameState, queue_size=1, latch=True)
        self.event_pub = rospy.Publisher("/chess/events", String, queue_size=20)
        self.clients = {}
        for color, ns in self.robot_ns.items():
            self.clients[color] = actionlib.SimpleActionClient(
                ns.rstrip("/") + "/execute_move", ExecuteMoveAction)

        self.new_game()
        rospy.Service("/chess/submit_move", SubmitMove, self.on_submit)
        rospy.Service("/chess/new_game", Trigger, self.on_new_game)
        rospy.loginfo("chess referee ready, robots: %s", self.robot_ns)

    # ----------------------------------------------------------------- state
    def new_game(self):
        with self.lock:
            self.board = chess.Board()
            self.physical = PhysicalState(initial_square_models(self.board))
            self.history = []
            self.status = GameState.STATUS_WAITING_FOR_MOVE
            self.status_text = "White to move"
            self.result = "*"
            self.draw_offer = ""
            self.pending = None
            self.last_san, self.last_uci = "", ""
            self.publish()

    def turn(self):
        return "white" if self.board.turn == chess.WHITE else "black"

    def event(self, text):
        rospy.loginfo(text)
        self.event_pub.publish(String(text))

    def publish(self):
        msg = GameState()
        msg.stamp = rospy.Time.now()
        msg.fen = self.board.fen()
        msg.turn = self.turn()
        msg.status = self.status
        msg.status_text = self.status_text
        msg.last_move_san = self.last_san
        msg.last_move_uci = self.last_uci
        msg.move_history_san = list(self.history)
        msg.result = self.result
        msg.in_check = self.board.is_check()
        msg.draw_offered_by = self.draw_offer
        msg.robot_colors = sorted(self.robot_ns.keys())
        self.state_pub.publish(msg)

    def finish(self, result, text):
        self.result = result
        self.status = GameState.STATUS_GAME_OVER
        self.status_text = text
        self.event("GAME OVER: %s (%s)" % (text, result))

    def check_game_end(self):
        outcome = self.board.outcome(claim_draw=False)
        if outcome is None:
            return False
        reason = outcome.termination.name.replace("_", " ").lower()
        if outcome.winner is None:
            self.finish("1/2-1/2", "draw by %s" % reason)
        else:
            winner = "white" if outcome.winner == chess.WHITE else "black"
            self.finish(outcome.result(), "%s wins by %s" % (winner, reason))
        return True

    # -------------------------------------------------------------- services
    def on_new_game(self, _req):
        with self.lock:
            if self.pending is not None:
                return TriggerResponse(False, "a robot is still moving")
            self.new_game()
        self.event("new game started")
        return TriggerResponse(True, "new game")

    def on_submit(self, req):
        color = req.color.strip().lower()
        text = req.move.strip()
        with self.lock:
            if color not in ("white", "black"):
                return SubmitMoveResponse(False, "unknown colour '%s'" % req.color)
            if self.status == GameState.STATUS_GAME_OVER:
                return SubmitMoveResponse(False, "the game is over (%s)" % self.result)
            if self.pending is not None:
                return SubmitMoveResponse(False, "a robot is still executing %s" % self.pending)

            command = text.lower()
            if command == "resign":
                winner = opposite(color)
                self.finish("1-0" if winner == "white" else "0-1",
                            "%s resigned, %s wins" % (color, winner))
                self.publish()
                return SubmitMoveResponse(True, "resigned")
            if command == "draw":
                return self.handle_draw(color)

            if color != self.turn():
                return SubmitMoveResponse(False, "it is %s's turn" % self.turn())
            try:
                move = parse_move(self.board, text)
            except ValueError as exc:
                return SubmitMoveResponse(False, str(exc))

            san = self.board.san(move)
            tasks, new_physical = plan_move(self.board, move, self.physical, self.geometry)
            self.draw_offer = ""

            if color not in self.clients:
                self.commit(move, san, new_physical)
                self.publish()
                return SubmitMoveResponse(True, "%s played %s" % (color, san))

            client = self.clients[color]
            if not client.wait_for_server(rospy.Duration(2.0)):
                return SubmitMoveResponse(False, "the %s robot is not available" % color)

            goal = ExecuteMoveGoal()
            goal.move_uci = move.uci()
            goal.tasks = [PieceTask(**t.as_dict()) for t in tasks]
            goal.occupied_squares = self.physical.occupied_squares()
            self.pending = san
            self.status = GameState.STATUS_ROBOT_MOVING
            self.status_text = "%s robot is playing %s (%d task%s)" % (
                color, san, len(tasks), "" if len(tasks) == 1 else "s")
            self.event(self.status_text)
            self.publish()
            client.send_goal(
                goal,
                done_cb=lambda st, res: self.on_robot_done(color, move, san,
                                                           new_physical, st, res),
                feedback_cb=lambda fb: self.event("[%s robot] %s: %s" % (
                    color, fb.stage, fb.text)))
            return SubmitMoveResponse(True, "%s accepted, robot is moving" % san)

    def handle_draw(self, color):
        if self.board.can_claim_draw():
            self.finish("1/2-1/2", "draw claimed by %s" % color)
            self.publish()
            return SubmitMoveResponse(True, "draw claimed")
        if self.draw_offer == opposite(color):
            self.finish("1/2-1/2", "draw agreed")
            self.publish()
            return SubmitMoveResponse(True, "draw agreed")
        self.draw_offer = color
        self.event("%s offers a draw" % color)
        self.publish()
        return SubmitMoveResponse(True, "draw offered - the opponent can answer 'draw'")

    # ------------------------------------------------------------ execution
    def commit(self, move, san, new_physical):
        self.board.push(move)
        self.physical = new_physical
        self.history.append(san)
        self.last_san, self.last_uci = san, move.uci()
        self.event("%s played %s" % (opposite(self.turn()), san))
        if not self.check_game_end():
            self.status = GameState.STATUS_WAITING_FOR_MOVE
            self.status_text = "%s to move%s" % (
                self.turn().capitalize(), " - CHECK" if self.board.is_check() else "")

    def on_robot_done(self, color, move, san, new_physical, status, result):
        with self.lock:
            self.pending = None
            if result is not None and result.success:
                self.commit(move, san, new_physical)
            else:
                why = result.message if result is not None else "no result"
                self.status = GameState.STATUS_ROBOT_ERROR
                self.status_text = ("%s robot failed to play %s: %s - check the board "
                                    "and submit the move again" % (color, san, why))
                self.event(self.status_text)
            self.publish()


if __name__ == "__main__":
    rospy.init_node("chess_referee")
    ChessReferee()
    rospy.spin()
