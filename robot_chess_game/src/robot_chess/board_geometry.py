"""Metric geometry of the floor chessboard.

Board frame: origin at the centre of the board, +x from the a-file towards
the h-file, +y from rank 1 (white) towards rank 8 (black).  The board frame
is placed in the map frame with (origin_x, origin_y, yaw), which makes it
possible to use the same code for the simulated arena and for a board taped
on the laboratory floor.

Pieces always stand at square centres, the robots drive on the lanes
between the squares (see road_network.py).
"""
import math

import chess


def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class BoardGeometry(object):

    PARAM_KEYS = ("square_size", "origin_x", "origin_y", "yaw", "piece_radius",
                  "graveyard_offset", "graveyard_spacing", "graveyard_slots",
                  "graveyard_approach", "ring_margin", "robot_length", "robot_width")

    def __init__(self, square_size=0.8, origin_x=0.0, origin_y=0.0, yaw=0.0,
                 piece_radius=0.09, graveyard_offset=4.7, graveyard_spacing=0.4,
                 graveyard_slots=15, graveyard_approach=0.4, ring_margin=0.5,
                 robot_length=0.32, robot_width=0.22):
        self.square_size = float(square_size)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.yaw = float(yaw)
        self.piece_radius = float(piece_radius)
        self.graveyard_offset = float(graveyard_offset)
        self.graveyard_spacing = float(graveyard_spacing)
        self.graveyard_slots = int(graveyard_slots)
        self.graveyard_approach = float(graveyard_approach)
        self.ring_margin = float(ring_margin)
        self.robot_length = float(robot_length)
        self.robot_width = float(robot_width)

    @classmethod
    def from_dict(cls, params):
        """Build from a (ROS parameter) dictionary; unknown keys are ignored."""
        return cls(**{k: params[k] for k in cls.PARAM_KEYS if k in params})

    # ------------------------------------------------------------- checks
    def robot_radius(self):
        """Circumscribed radius of the rectangular robot footprint."""
        return math.hypot(self.robot_length / 2.0, self.robot_width / 2.0)

    def validate(self, min_clearance=0.03):
        """Raise ValueError if the robot cannot drive/turn safely on the board.

        Checked clearances (robot circle or half width vs. piece surface):
          * turning in place on a square-side midpoint (the approach point),
            between the two pieces it separates;
          * driving along a lane;
          * turning on the ring road next to the outer pieces;
          * turning at a graveyard approach point next to the neighbour slots.
        """
        r, p = self.robot_radius(), self.piece_radius
        s2 = self.square_size / 2.0
        checks = {
            "turn at a square-side midpoint": s2 - p - r,
            "drive along a lane": s2 - p - self.robot_width / 2.0,
            "turn on the ring road": self.ring_margin + s2 - p - r,
            "turn at a graveyard slot": math.hypot(self.graveyard_spacing,
                                                   self.graveyard_approach) - p - r,
            "graveyard outside the ring road":
                self.graveyard_offset - self.graveyard_approach
                - (self.board_half_size() + self.ring_margin) - 0.2,
            "graveyard slots beside the board":
                self.board_half_size() + self.ring_margin
                - (self.graveyard_slots - 1) / 2.0 * self.graveyard_spacing - 0.2,
        }
        bad = ["%s (%.3f m)" % (k, v) for k, v in checks.items() if v < min_clearance]
        if bad:
            raise ValueError("board geometry too tight: " + ", ".join(bad))
        return checks

    # ------------------------------------------------------------------ frames
    def board_to_map(self, bx, by):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return (self.origin_x + c * bx - s * by, self.origin_y + s * bx + c * by)

    def map_to_board(self, mx, my):
        dx, dy = mx - self.origin_x, my - self.origin_y
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return (c * dx + s * dy, -s * dx + c * dy)

    def board_yaw_to_map(self, yaw):
        return normalize_angle(yaw + self.yaw)

    # ----------------------------------------------------------------- squares
    def square_center_board(self, square):
        if isinstance(square, str):
            square = chess.parse_square(square)
        f, r = chess.square_file(square), chess.square_rank(square)
        return ((f - 3.5) * self.square_size, (r - 3.5) * self.square_size)

    def square_center(self, square):
        """Centre of a square in the map frame."""
        return self.board_to_map(*self.square_center_board(square))

    def square_at(self, mx, my):
        """Square name under a map point, or None when off the board."""
        bx, by = self.map_to_board(mx, my)
        f = int(math.floor(bx / self.square_size + 4.0))
        r = int(math.floor(by / self.square_size + 4.0))
        if 0 <= f < 8 and 0 <= r < 8:
            return chess.square_name(chess.square(f, r))
        return None

    def board_half_size(self):
        return 4.0 * self.square_size

    # --------------------------------------------------------------- graveyard
    def graveyard_slot_board(self, color, index):
        """Captured white pieces are stored left of the board (a-file side),
        captured black pieces right of it (h-file side)."""
        index = max(0, min(int(index), self.graveyard_slots - 1))
        x = -self.graveyard_offset if color == "white" else self.graveyard_offset
        y = (index - (self.graveyard_slots - 1) / 2.0) * self.graveyard_spacing
        return (x, y)

    def graveyard_slot(self, color, index):
        return self.board_to_map(*self.graveyard_slot_board(color, index))
