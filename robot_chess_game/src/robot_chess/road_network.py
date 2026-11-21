"""Road network the robots drive on, and shortest path planning on it.

Pieces always stand on square centres, therefore the lines *between* the
squares are free lanes: with 0.8 m squares and 0.18 m pieces a lane is
0.62 m wide, the LIMO is 0.22 m wide.  The robots only drive on these
lanes, plus a ring road around the board and short spurs to the home poses
and to the graveyard slots:

        home (black)
             |
    +--------+--------+   ring road (board half size + ring_margin)
    |  +--+--+--+--+  |
    |  |  |  |  |  |  |   board grid lines (lanes between the squares)
  G-+--+--+--+--+--+--+-G graveyard spurs (one per slot)
    |  +--+--+--+--+  |
    +--------+--------+
             |
        home (white)

Nodes are the grid line intersections, the midpoints of every square side
(the approach points: the robot stands there and faces the square), the
ring nodes and the spur ends.  Rotating in place is safe on every node:
BoardGeometry.validate() checks that the robot's circumscribed circle fits
between the two pieces next to a square-side midpoint.

All computations are done in the board frame; inputs and outputs of the
public methods are in the map frame.
"""
import heapq
import math

import chess

from robot_chess.board_geometry import normalize_angle


def _key(x, y):
    return (round(x, 4), round(y, 4))


class RoadNetwork(object):

    def __init__(self, geometry, homes=None):
        """homes: {"white": (x, y, yaw), ...} in the *board* frame."""
        self.g = geometry
        self.homes = dict(homes or {})
        self.adj = {}
        self._build()

    # ------------------------------------------------------------ building
    def _add_edge(self, a, b):
        a, b = _key(*a), _key(*b)
        if a == b:
            return
        d = math.hypot(a[0] - b[0], a[1] - b[1])
        self.adj.setdefault(a, {})[b] = d
        self.adj.setdefault(b, {})[a] = d

    def _add_line(self, points):
        """Connect consecutive points of a straight line."""
        pts = sorted({_key(*p) for p in points})
        for a, b in zip(pts, pts[1:]):
            self._add_edge(a, b)

    def _build(self):
        g = self.g
        s = g.square_size
        half = g.board_half_size()
        ring = half + g.ring_margin
        lines = [(i - 4) * s for i in range(9)]          # grid lines
        mids = [(i - 3.5) * s for i in range(8)]         # square centres

        graveyard_ys = [g.graveyard_slot_board("white", i)[1]
                        for i in range(g.graveyard_slots)]
        home_xs = [h[0] for h in self.homes.values()]

        # Lanes on the board, extended to the ring road.
        for c in lines:
            along = lines + mids + [-ring, ring]
            self._add_line([(c, v) for v in along])      # vertical lane x = c
            self._add_line([(v, c) for v in along])      # horizontal lane y = c

        # Ring road.
        ring_x = sorted(set(lines + [-ring, ring] + home_xs))
        ring_y = sorted(set(lines + [-ring, ring] + graveyard_ys))
        self._add_line([(x, -ring) for x in ring_x])
        self._add_line([(x, ring) for x in ring_x])
        self._add_line([(-ring, y) for y in ring_y])
        self._add_line([(ring, y) for y in ring_y])

        # Graveyard spurs: from the ring straight out to the approach point.
        for color in ("white", "black"):
            for i in range(g.graveyard_slots):
                node, _ = self._graveyard_approach_board(color, i)
                side = math.copysign(ring, node[0])
                self._add_edge((side, node[1]), node)

        # Home spurs.
        for hx, hy, _ in self.homes.values():
            side = math.copysign(ring, hy)
            self._add_edge((hx, side), (hx, hy))

    # ----------------------------------------------------------- approach
    def _graveyard_approach_board(self, color, index):
        sx, sy = self.g.graveyard_slot_board(color, index)
        ax = sx - math.copysign(self.g.graveyard_approach, sx)
        yaw = 0.0 if sx > 0 else math.pi                 # face the slot
        return _key(ax, sy), yaw

    def _square_approaches_board(self, square):
        cx, cy = self.g.square_center_board(square)
        d = self.g.square_size / 2.0
        out = []
        for ux, uy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            node = _key(cx + ux * d, cy + uy * d)
            out.append((node, math.atan2(-uy, -ux)))    # face the square centre
        return out

    def approaches(self, label):
        """Approach nodes for a square name or a "graveyard-<color>-<n>" label,
        as a list of (node_board, yaw_board)."""
        if label.startswith("graveyard-"):
            _, color, idx = label.split("-")
            return [self._graveyard_approach_board(color, int(idx))]
        chess.parse_square(label)                          # validates the name
        return self._square_approaches_board(label)

    def home_board(self, color):
        hx, hy, yaw = self.homes[color]
        return _key(hx, hy), yaw

    # ------------------------------------------------------------ planning
    def nearest_node(self, bx, by):
        return min(self.adj, key=lambda n: math.hypot(n[0] - bx, n[1] - by))

    def _dijkstra(self, start):
        dist = {start: 0.0}
        prev = {}
        heap = [(0.0, start)]
        while heap:
            d, node = heapq.heappop(heap)
            if d > dist[node]:
                continue
            for nb, w in self.adj[node].items():
                nd = d + w
                if nd < dist.get(nb, float("inf")):
                    dist[nb] = nd
                    prev[nb] = node
                    heapq.heappush(heap, (nd, nb))
        return dist, prev

    @staticmethod
    def _path(prev, start, goal):
        path = [goal]
        while path[-1] != start:
            path.append(prev[path[-1]])
        return path[::-1]

    @staticmethod
    def simplify(path):
        """Drop intermediate nodes of straight segments (no stop there)."""
        if len(path) < 3:
            return list(path)
        out = [path[0]]
        for a, b, c in zip(path, path[1:], path[2:]):
            cross = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0])
            if abs(cross) > 1e-6:
                out.append(b)
        out.append(path[-1])
        return out

    def plan(self, robot_xy_map, goals_board):
        """Shortest route from the robot to the best of several goals.

        goals_board: list of (node, yaw) in the board frame.
        Returns (waypoints_map, final_yaw_map, length) with the simplified
        list of map-frame waypoints (first = nearest network node).
        """
        bx, by = self.g.map_to_board(*robot_xy_map)
        start = self.nearest_node(bx, by)
        dist, prev = self._dijkstra(start)
        best = None
        for node, yaw in goals_board:
            if node in dist and (best is None or dist[node] < best[0]):
                best = (dist[node], node, yaw)
        if best is None:
            raise ValueError("no route to any goal")
        length, goal, yaw = best
        path = self.simplify(self._path(prev, start, goal))
        waypoints = [self.g.board_to_map(x, y) for x, y in path]
        return waypoints, self.g.board_yaw_to_map(yaw), length


def path_length(waypoints):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(waypoints, waypoints[1:]))


def heading_error(x, y, yaw, tx, ty):
    return normalize_angle(math.atan2(ty - y, tx - x) - yaw)
