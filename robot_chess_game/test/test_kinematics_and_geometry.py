import math
import os

import numpy as np
import pytest
import yaml

from robot_chess.board_geometry import BoardGeometry
from robot_chess.kinematics import MyCobotKinematics, interpolate_line

CONFIG = os.path.join(os.path.dirname(__file__), "..", "config", "player_sim.yaml")


def sim_geometry():
    with open(CONFIG) as f:
        return BoardGeometry.from_dict(yaml.safe_load(f)["board"])


def test_zero_pose_matches_urdf():
    """At zero angles the myCobot stands upright, flange 0.411 m above joint1."""
    k = MyCobotKinematics(tcp_offset=0.0, mount=(0, 0, 0))
    f = k.flange([0.0] * 6)
    assert f[:3, 3] == pytest.approx([0.0456, -0.0646, 0.4111], abs=1e-3)


def test_ik_round_trip():
    k = MyCobotKinematics()
    for target in ([0.18, 0.05, 0.24], [0.12, -0.06, 0.30], [0.22, 0.0, 0.23]):
        q = k.ik_down(target)
        assert q is not None, target
        p, axis = k.tcp(q)
        assert p == pytest.approx(target, abs=1e-3)
        assert axis == pytest.approx([0, 0, -1], abs=5e-3)
        assert all(abs(v) <= math.radians(175) + 1e-9 for v in q)


def test_unreachable_target():
    assert MyCobotKinematics().ik_down([0.6, 0.0, 0.2]) is None


def test_ik_prefers_solution_close_to_seed():
    k = MyCobotKinematics()
    q1 = k.ik_down([0.18, 0.0, 0.24])
    q2 = k.ik_down([0.185, 0.0, 0.24], seed=q1)
    assert np.abs(np.array(q2) - np.array(q1)).max() < 0.2


def test_board_frame_for_white_robot():
    g = BoardGeometry(board_x=0.17, board_y=0.0, board_z=0.22, board_yaw=-math.pi / 2)
    # rank 1 is nearest to the robot, the h-file to its right (-y)
    a1, h8 = g.to_robot(*g.square_center_board("a1")), g.to_robot(*g.square_center_board("h8"))
    assert a1[0] < h8[0]
    assert a1[1] > h8[1]
    assert g.square_at_board(*g.square_center_board("e4")) == 28


def test_sim_layout_fully_reachable():
    """Every square and graveyard slot at grasp and travel height, and the
    camera observation pose, are reachable with the tool pointing down."""
    g, k = sim_geometry(), MyCobotKinematics()
    for label in g.all_labels():
        for p in (g.grasp_point(label), g.travel_point(label)):
            assert k.ik_down(p) is not None, label
    assert k.ik_down(g.observe_point()) is not None


def test_graveyard_slots_distinct_and_off_board():
    g = sim_geometry()
    slots = [g.graveyard_slot_board(i) for i in range(g.graveyard_slots)]
    assert len(set(slots)) == g.graveyard_slots
    for x, y in slots:
        assert g.square_at_board(x, y) is None


def test_interpolate_line():
    pts = interpolate_line([0, 0, 0], [0.1, 0, 0], step=0.03)
    assert len(pts) == 4
    assert pts[-1] == pytest.approx([0.1, 0, 0])
