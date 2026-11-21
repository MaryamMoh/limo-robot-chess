"""Where the chessboard is, seen from the robot.

Board frame: origin at the centre of the board, on the playing surface;
+x from the a-file to the h-file, +y from rank 1 to rank 8, +z up.

The board pose is given in the robot's base_footprint frame
(board_x, board_y, board_z, board_yaw).  For a robot playing white and
facing the board, the board's +y (towards black) is the robot's forward
direction, i.e. board_yaw = -pi/2; for black board_yaw = +pi/2.

Captured opponent pieces are put on a grid of "graveyard" slots beside the
board (also in board coordinates) that must be inside the arm's reach.
"""
import math

import chess
import numpy as np


class BoardGeometry(object):

    PARAM_KEYS = ("square_size", "board_x", "board_y", "board_z", "board_yaw",
                  "graveyard_x", "graveyard_y", "graveyard_dx", "graveyard_dy",
                  "graveyard_rows", "graveyard_slots", "grasp_height",
                  "travel_height", "observe_x", "observe_y", "observe_height")

    def __init__(self, square_size=0.02, board_x=0.17, board_y=0.0, board_z=0.22,
                 board_yaw=-math.pi / 2, graveyard_x=0.095, graveyard_y=-0.07,
                 graveyard_dx=0.02, graveyard_dy=0.02, graveyard_rows=5,
                 graveyard_slots=15, grasp_height=0.012, travel_height=0.06,
                 observe_x=0.0, observe_y=-0.04, observe_height=0.13):
        self.square_size = float(square_size)
        self.board_x = float(board_x)
        self.board_y = float(board_y)
        self.board_z = float(board_z)
        self.board_yaw = float(board_yaw)
        self.graveyard_x = float(graveyard_x)
        self.graveyard_y = float(graveyard_y)
        self.graveyard_dx = float(graveyard_dx)
        self.graveyard_dy = float(graveyard_dy)
        self.graveyard_rows = int(graveyard_rows)
        self.graveyard_slots = int(graveyard_slots)
        self.grasp_height = float(grasp_height)
        self.travel_height = float(travel_height)
        self.observe_x = float(observe_x)
        self.observe_y = float(observe_y)
        self.observe_height = float(observe_height)

    @classmethod
    def from_dict(cls, params):
        return cls(**{k: params[k] for k in cls.PARAM_KEYS if k in params})

    # ----------------------------------------------------------- transforms
    def board_to_robot_matrix(self):
        """4x4 transform board frame -> base_footprint."""
        c, s = math.cos(self.board_yaw), math.sin(self.board_yaw)
        m = np.eye(4)
        m[:2, :2] = [[c, -s], [s, c]]
        m[:3, 3] = [self.board_x, self.board_y, self.board_z]
        return m

    def to_robot(self, bx, by, bz=0.0):
        p = self.board_to_robot_matrix() @ np.array([bx, by, bz, 1.0])
        return p[:3]

    # --------------------------------------------------------------- squares
    def square_center_board(self, square):
        if isinstance(square, str):
            square = chess.parse_square(square)
        f, r = chess.square_file(square), chess.square_rank(square)
        return ((f - 3.5) * self.square_size, (r - 3.5) * self.square_size)

    def square_at_board(self, bx, by):
        f = int(math.floor(bx / self.square_size + 4.0))
        r = int(math.floor(by / self.square_size + 4.0))
        if 0 <= f < 8 and 0 <= r < 8:
            return chess.square(f, r)
        return None

    def graveyard_slot_board(self, index):
        index = max(0, min(int(index), self.graveyard_slots - 1))
        col, row = divmod(index, self.graveyard_rows)
        return (self.graveyard_x + col * self.graveyard_dx,
                self.graveyard_y + row * self.graveyard_dy)

    def location_board(self, label):
        """Board-frame (x, y) of a square name or "graveyard-<n>"."""
        if label.startswith("graveyard-"):
            return self.graveyard_slot_board(int(label.split("-")[1]))
        return self.square_center_board(label)

    # ------------------------------------------------------- arm targets
    def grasp_point(self, label):
        """TCP target (base_footprint) for grasping/releasing at a location."""
        bx, by = self.location_board(label)
        return self.to_robot(bx, by, self.grasp_height)

    def travel_point(self, label):
        bx, by = self.location_board(label)
        return self.to_robot(bx, by, self.travel_height)

    def observe_point(self):
        """TCP position (tool pointing down) from which the wide-angle wrist
        camera sees the whole board and the graveyard.  It is slightly on the
        robot's side of the board centre: the arm cannot hold the camera high
        enough above the centre itself."""
        return self.to_robot(self.observe_x, self.observe_y, self.observe_height)

    def all_labels(self):
        return ([chess.square_name(s) for s in chess.SQUARES] +
                ["graveyard-%d" % i for i in range(self.graveyard_slots)])
