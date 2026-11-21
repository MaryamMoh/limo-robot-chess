"""Metric geometry of the floor chessboard.

Board frame: origin at the centre of the board, +x from the a-file towards
the h-file, +y from rank 1 (white) towards rank 8 (black).  The board frame
is placed in the map frame with (origin_x, origin_y, yaw), which makes it
possible to use the same code for the simulated arena and for a board taped
on the laboratory floor.

Pieces always stand at square centres.  With a square size of 0.7 m and a
piece diameter of 0.18 m the lines between the squares form a free road
network 0.52 m wide, wider than the LIMO (0.22 m), so a robot can reach
any square no matter how crowded the board is.
"""
import math

import chess

from robot_chess.pieces import PIECE_RADIUS


def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class BoardGeometry(object):

    PARAM_KEYS = ("square_size", "origin_x", "origin_y", "yaw",
                  "graveyard_offset", "graveyard_spacing", "graveyard_slots",
                  "workspace_half_extent", "orthogonal_approach",
                  "robot_radius", "clearance_margin")

    def __init__(self, square_size=0.7, origin_x=0.0, origin_y=0.0, yaw=0.0,
                 graveyard_offset=4.5, graveyard_spacing=0.35,
                 graveyard_slots=15, workspace_half_extent=4.8,
                 orthogonal_approach=0.42, robot_radius=0.20,
                 clearance_margin=0.05):
        self.square_size = float(square_size)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.yaw = float(yaw)
        self.graveyard_offset = float(graveyard_offset)
        self.graveyard_spacing = float(graveyard_spacing)
        self.graveyard_slots = int(graveyard_slots)
        self.workspace_half_extent = float(workspace_half_extent)
        self.orthogonal_approach = float(orthogonal_approach)
        # Standing exactly on a square corner keeps the robot clear of all
        # four pieces around that corner.
        self.diagonal_approach = self.square_size / math.sqrt(2.0)
        self.robot_radius = float(robot_radius)
        self.clearance = self.robot_radius + PIECE_RADIUS + float(clearance_margin)

    @classmethod
    def from_dict(cls, params):
        """Build from a (ROS parameter) dictionary; unknown keys are ignored."""
        return cls(**{k: params[k] for k in cls.PARAM_KEYS if k in params})

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

    # --------------------------------------------------------------- approach
    def approach_poses(self, target_xy, obstacle_xys, robot_xy=None):
        """Candidate robot poses from which a piece at target_xy can be
        reached (picked up or put down).

        target_xy, obstacle_xys and robot_xy are map-frame points.  A
        candidate is kept only if the robot's circumscribed circle keeps
        `clearance` from every other piece and the pose lies inside the
        workspace.  Returns a list of (x, y, yaw) in the map frame, robot
        facing the target, sorted by preference: orthogonal approaches first
        (they bring the camera closer), then by distance to robot_xy.
        """
        tbx, tby = self.map_to_board(*target_xy)
        obstacles = [self.map_to_board(*p) for p in obstacle_xys]
        obstacles = [p for p in obstacles
                     if math.hypot(p[0] - tbx, p[1] - tby) > 1e-3]
        rb = self.map_to_board(*robot_xy) if robot_xy is not None else None

        candidates = []
        for kind, dist, angles in (
                (0, self.orthogonal_approach, (0, 90, 180, 270)),
                (1, self.diagonal_approach, (45, 135, 225, 315))):
            for deg in angles:
                a = math.radians(deg)
                px, py = tbx + dist * math.cos(a), tby + dist * math.sin(a)
                if max(abs(px), abs(py)) > self.workspace_half_extent:
                    continue
                if any(math.hypot(px - ox, py - oy) < self.clearance
                       for ox, oy in obstacles):
                    continue
                facing = normalize_angle(a + math.pi)
                travel = math.hypot(px - rb[0], py - rb[1]) if rb else 0.0
                candidates.append((kind, travel, px, py, facing))

        candidates.sort(key=lambda c: (c[0], c[1]))
        out = []
        for _, _, px, py, facing in candidates:
            mx, my = self.board_to_map(px, py)
            out.append((mx, my, self.board_yaw_to_map(facing)))
        return out
