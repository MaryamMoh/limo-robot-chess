import math
import os

import chess
import pytest
import yaml

from robot_chess.board_geometry import BoardGeometry
from robot_chess.road_network import RoadNetwork, path_length

CONFIG_DIR = os.path.join(os.path.dirname(__file__), "..", "config")


def network(config="board_sim.yaml"):
    with open(os.path.join(CONFIG_DIR, config)) as f:
        cfg = yaml.safe_load(f)
    g = BoardGeometry.from_dict(cfg["board"])
    homes = {c: (h["x"], h["y"], h["yaw"]) for c, h in cfg["home"].items()}
    return g, RoadNetwork(g, homes)


def _min_distance_to_segment(p, a, b):
    ax, ay = a
    bx, by = b
    px, py = p
    dx, dy = bx - ax, by - ay
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)))
    return math.hypot(ax + t * dx - px, ay + t * dy - py)


@pytest.mark.parametrize("config", ["board_sim.yaml", "board_real.yaml"])
def test_every_edge_keeps_clear_of_every_possible_piece(config):
    """No road segment passes closer to a square centre or a graveyard slot
    than half a robot width + piece radius (pieces can only stand there)."""
    g, net = network(config)
    pieces = [g.square_center_board(sq) for sq in chess.SQUARES]
    for color in ("white", "black"):
        pieces += [g.graveyard_slot_board(color, i) for i in range(g.graveyard_slots)]
    limit = g.robot_width / 2.0 + g.piece_radius
    for a, nbs in net.adj.items():
        for b in nbs:
            for p in pieces:
                assert _min_distance_to_segment(p, a, b) > limit, (a, b, p)


@pytest.mark.parametrize("config", ["board_sim.yaml", "board_real.yaml"])
def test_every_target_reachable_from_both_homes(config):
    g, net = network(config)
    labels = [chess.square_name(s) for s in chess.SQUARES]
    labels += ["graveyard-%s-%d" % (c, i) for c in ("white", "black")
               for i in range(g.graveyard_slots)]
    for color in ("white", "black"):
        hx, hy, _ = net.homes[color]
        start = g.board_to_map(hx, hy)
        for label in labels:
            waypoints, _, length = net.plan(start, net.approaches(label))
            assert length > 0 and len(waypoints) >= 2, label


def test_route_home_to_e2_uses_the_lane_between_d_and_e_files():
    g, net = network()
    start = g.board_to_map(0.0, -4.5)
    waypoints, yaw, length = net.plan(start, net.approaches("e2"))
    # straight up the x = 0 lane, then turn east to face e2
    assert waypoints == [pytest.approx((0.0, -4.5)), pytest.approx((0.0, -2.0))]
    assert yaw == pytest.approx(0.0)
    assert length == pytest.approx(2.5)


def test_graveyard_route_faces_the_slot():
    g, net = network()
    start = g.board_to_map(0.4, 0.0)                    # somewhere on the board
    waypoints, yaw, _ = net.plan(start, net.approaches("graveyard-black-0"))
    slot = g.graveyard_slot("black", 0)
    end = waypoints[-1]
    assert end == pytest.approx((slot[0] - g.graveyard_approach, slot[1]))
    assert yaw == pytest.approx(0.0)


def test_path_is_simplified_to_corners_only():
    g, net = network()
    waypoints, _, length = net.plan((0.0, -4.5), net.approaches("h8"))
    assert path_length(waypoints) == pytest.approx(length)
    for a, b in zip(waypoints, waypoints[1:]):
        # every segment is axis parallel
        assert abs(a[0] - b[0]) < 1e-6 or abs(a[1] - b[1]) < 1e-6


def test_rotated_board_routes_in_map_frame():
    g = BoardGeometry(origin_x=10.0, origin_y=5.0, yaw=math.pi / 2)
    net = RoadNetwork(g, {"white": (0.0, -4.5, math.pi / 2)})
    start = g.board_to_map(0.0, -4.5)
    waypoints, yaw, _ = net.plan(start, net.approaches("e2"))
    assert waypoints[-1] == pytest.approx(g.board_to_map(0.0, -2.0))
    assert yaw == pytest.approx(math.pi / 2)


def test_unknown_label_rejected():
    _, net = network()
    with pytest.raises(ValueError):
        net.approaches("z9")


def test_robot_between_nodes_stays_on_its_lane():
    """Off-node and slightly off-lane starts are projected onto the lane:
    the route never cuts diagonally through a square."""
    g, net = network()
    for start in [(0.0, 2.0), (0.19, 2.22), (0.05, 1.3), (-0.07, 2.1)]:
        waypoints, _, _ = net.plan(start, net.approaches("e5"))
        for a, b in zip(waypoints, waypoints[1:]):
            assert abs(a[0] - b[0]) < 1e-6 or abs(a[1] - b[1]) < 1e-6, (start, waypoints)
        entry = waypoints[0]
        assert math.hypot(entry[0] - start[0], entry[1] - start[1]) <= 0.25
