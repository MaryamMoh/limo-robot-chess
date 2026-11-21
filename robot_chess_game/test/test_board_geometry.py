import math

import chess
import pytest

from robot_chess.board_geometry import BoardGeometry
from robot_chess.pieces import initial_square_models


def test_square_centres_and_inverse():
    g = BoardGeometry(square_size=0.7)
    assert g.square_center("a1") == pytest.approx((-2.45, -2.45))
    assert g.square_center("h8") == pytest.approx((2.45, 2.45))
    assert g.square_center("e4") == pytest.approx((0.35, -0.35))
    for sq in chess.SQUARES:
        x, y = g.square_center(sq)
        assert g.square_at(x, y) == chess.square_name(sq)
    assert g.square_at(10.0, 0.0) is None


def test_rotated_board_round_trip():
    g = BoardGeometry(square_size=0.4, origin_x=1.0, origin_y=-2.0, yaw=math.pi / 2)
    x, y = g.square_center("a1")
    # rotated by +90 deg: board -x/-y corner maps to map (+x, -y) side
    assert (x, y) == pytest.approx((1.0 + 1.4, -2.0 - 1.4))
    bx, by = g.map_to_board(x, y)
    assert (bx, by) == pytest.approx(g.square_center_board("a1"))
    for sq in chess.SQUARES:
        assert g.square_at(*g.square_center(sq)) == chess.square_name(sq)


def test_graveyard_slots_are_on_opposite_sides_and_distinct():
    g = BoardGeometry()
    white = [g.graveyard_slot("white", i) for i in range(15)]
    black = [g.graveyard_slot("black", i) for i in range(15)]
    assert all(x < -g.board_half_size() for x, _ in white)
    assert all(x > g.board_half_size() for x, _ in black)
    assert len(set(white)) == 15
    assert max(abs(y) for _, y in white) < g.board_half_size()


def _occupied_points(g, board):
    return [g.square_center(sq) for sq in board.piece_map()]


def test_every_square_reachable_in_start_position():
    """Each piece of the opening position has at least one approach pose."""
    g = BoardGeometry()
    board = chess.Board()
    obstacles = _occupied_points(g, board)
    for sq in board.piece_map():
        poses = g.approach_poses(g.square_center(sq), obstacles)
        assert poses, chess.square_name(sq)


def test_corner_approach_on_full_board():
    """Even when all 64 squares are occupied the corner approach is free."""
    g = BoardGeometry()
    obstacles = [g.square_center(sq) for sq in chess.SQUARES]
    poses = g.approach_poses(g.square_center("d4"), obstacles)
    assert len(poses) == 4
    for x, y, yaw in poses:
        tx, ty = g.square_center("d4")
        assert math.hypot(x - tx, y - ty) == pytest.approx(g.diagonal_approach)
        # robot faces the target
        assert math.atan2(ty - y, tx - x) == pytest.approx(yaw)


def test_orthogonal_preferred_and_sorted_by_travel():
    g = BoardGeometry()
    target = g.square_center("e4")
    poses = g.approach_poses(target, [], robot_xy=(0.35, -4.0))
    assert len(poses) == 8
    # nearest orthogonal pose is the one south of e4
    assert poses[0][:2] == pytest.approx((0.35, -0.35 - g.orthogonal_approach))
    assert poses[0][2] == pytest.approx(math.pi / 2)


def test_graveyard_approach_from_inside():
    g = BoardGeometry()
    slot = g.graveyard_slot("black", 3)
    neighbours = [g.graveyard_slot("black", i) for i in range(15) if i != 3]
    poses = g.approach_poses(slot, neighbours)
    assert poses
    # robot stands between board and graveyard, facing +x
    assert poses[0][0] < slot[0]
    assert poses[0][2] == pytest.approx(0.0)


def test_initial_models_unique():
    models = initial_square_models()
    assert len(models) == 32
    assert len(set(models.values())) == 32
    assert models["g1"] == "white_knight_g"
    assert models["e8"] == "black_king_e"
