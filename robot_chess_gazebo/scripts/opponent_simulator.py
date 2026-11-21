#!/usr/bin/env python3
"""Stand-in for the opponent's robot (simulation only).

In the laboratory the other side is played by another team's arm running
their own code.  To test my player without it, this node plays the other
side the way an independent robot would:

  * it knows nothing about my player node - it only looks at the board
    (Gazebo ground truth on /chess_sim/occupancy) and recognises my moves
    with the same legal-move matching;
  * a move of mine is accepted when the board has been stable for
    ~settle_time seconds (my arm has finished);
  * it then thinks (~think_time) and moves its pieces one after the other
    (captured pieces go off the board behind its back rank).

Parameters: ~board, ~color (side it plays), ~strategy (engine | random),
~think_time, ~seed
"""
import math
import random

import chess
import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import Pose
from std_msgs.msg import String

from robot_chess.board_geometry import BoardGeometry
from robot_chess.engine import InternalEngine
from robot_chess.move_inference import infer_move
from robot_chess.piece_models import piece_sdf
from robot_chess.pieces import PIECE_TYPE_NAME, chess_color


class OpponentSimulator(object):

    def __init__(self):
        self.g = BoardGeometry.from_dict(rospy.get_param("~board", {}))
        self.color = rospy.get_param("~color", "black")
        self.strategy = rospy.get_param("~strategy", "engine")
        self.think_time = rospy.get_param("~think_time", 2.0)
        self.settle_time = rospy.get_param("~settle_time", 3.0)
        self.piece_delay = rospy.get_param("~piece_delay", 1.0)
        self.rng = random.Random(rospy.get_param("~seed", 1))
        self.engine = InternalEngine(time_limit=self.think_time, max_depth=3)
        self.board = chess.Board()
        self.captured = 0
        self.promotions = 0
        self.occupancy = None
        self.occupancy_since = rospy.Time.now()
        self.models = {}

        self.state_pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=10)
        self.log_pub = rospy.Publisher("/chess_sim/opponent", String, queue_size=10, latch=True)
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        self.spawn = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self.delete = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.on_models, queue_size=1)
        rospy.Subscriber("/chess_sim/occupancy", String, self.on_occupancy, queue_size=1)

    def on_models(self, msg):
        self.models = {n: p for n, p in zip(msg.name, msg.pose)
                       if n.startswith(("white_", "black_"))}

    def on_occupancy(self, msg):
        if msg.data != self.occupancy:
            self.occupancy = msg.data
            self.occupancy_since = rospy.Time.now()

    def say(self, text):
        rospy.loginfo("[opponent] %s", text)
        self.log_pub.publish(String(text))

    # ----------------------------------------------------------- pieces
    def model_on(self, square):
        bx, by = self.g.square_center_board(square)
        for name, pose in self.models.items():
            px, py = self._board_xy(pose)
            if math.hypot(px - bx, py - by) < self.g.square_size * 0.4 and \
                    abs(pose.position.z - self.g.board_z) < 0.005:
                return name
        return None

    def _board_xy(self, pose):
        dx, dy = pose.position.x - self.g.board_x, pose.position.y - self.g.board_y
        c, s = math.cos(self.g.board_yaw), math.sin(self.g.board_yaw)
        return c * dx + s * dy, -s * dx + c * dy

    def place(self, name, bx, by):
        p = self.g.to_robot(bx, by)
        st = ModelState()
        st.model_name = name
        st.reference_frame = "world"
        st.pose.position.x, st.pose.position.y, st.pose.position.z = p[0], p[1], p[2]
        st.pose.orientation.w = 1.0
        for _ in range(3):
            self.state_pub.publish(st)
            rospy.sleep(0.05)
        rospy.sleep(self.piece_delay)

    def capture_area(self):
        """Off the board behind my back rank, in two rows."""
        s = self.g.square_size
        side = 1.0 if self.color == "black" else -1.0
        col, row = divmod(self.captured, 2)
        self.captured += 1
        return ((col - 3.5) * s, side * (4 * s + 0.03 + row * s))

    def execute(self, move):
        b = self.board
        captured = None
        if b.is_en_passant(move):
            captured = chess.square(chess.square_file(move.to_square),
                                    chess.square_rank(move.from_square))
        elif b.is_capture(move):
            captured = move.to_square
        if captured is not None:
            victim = self.model_on(captured)
            if victim:
                self.place(victim, *self.capture_area())
        mover = self.model_on(move.from_square)
        self.place(mover, *self.g.square_center_board(move.to_square))
        if move.promotion:
            self.delete(model_name=mover)
            self.promotions += 1
            ptype = PIECE_TYPE_NAME[move.promotion]
            name = "%s_%s_opp_promoted_%d" % (self.color, ptype, self.promotions)
            p = self.g.to_robot(*self.g.square_center_board(move.to_square))
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = p[0], p[1], p[2]
            pose.orientation.w = 1.0
            self.spawn(model_name=name, model_xml=piece_sdf(name, self.color, ptype),
                       robot_namespace="", initial_pose=pose, reference_frame="world")
        if b.is_castling(move):
            rank = chess.square_rank(move.from_square)
            rf, rt = ((7, 5) if b.is_kingside_castling(move) else (0, 3))
            rook = self.model_on(chess.square(rf, rank))
            self.place(rook, *self.g.square_center_board(chess.square(rt, rank)))

    # ------------------------------------------------------------- game
    def choose(self):
        moves = list(self.board.legal_moves)
        if self.strategy == "random":
            return self.rng.choice(moves)
        result = self.engine.choose(self.board)
        return result.move

    def run(self):
        rate = rospy.Rate(4)
        mine = chess_color(self.color)
        while not rospy.is_shutdown() and not self.board.is_game_over():
            if self.board.turn == mine:
                rospy.sleep(self.think_time)
                move = self.choose()
                san = self.board.san(move)
                self.execute(move)
                self.board.push(move)
                self.say("played %s" % san)
                rospy.sleep(self.settle_time)
            elif self.occupancy is not None and \
                    (rospy.Time.now() - self.occupancy_since).to_sec() > self.settle_time:
                result = infer_move(self.board, self.occupancy)
                if result.status == "move":
                    self.say("saw the opponent play %s" % self.board.san(result.move))
                    self.board.push(result.move)
            rate.sleep()
        self.say("game over: %s" % self.board.result(claim_draw=True))


if __name__ == "__main__":
    rospy.init_node("opponent_simulator")
    OpponentSimulator().run()
