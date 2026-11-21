import math
import os

import chess
import pytest
import yaml

from robot_chess.board_geometry import BoardGeometry
from robot_chess.pieces import initial_square_models

CONFIG_DIR = os.path.join(os.path.dirname(__file__), "..", "config")


def load_config(name):
    with open(os.path.join(CONFIG_DIR, name)) as f:
        return yaml.safe_load(f)


def test_square_centres_and_inverse():
    g = BoardGeometry(square_size=0.8)
    assert g.square_center("a1") == pytest.approx((-2.8, -2.8))
    assert g.square_center("h8") == pytest.approx((2.8, 2.8))
    assert g.square_center("e4") == pytest.approx((0.4, -0.4))
    for sq in chess.SQUARES:
        x, y = g.square_center(sq)
        assert g.square_at(x, y) == chess.square_name(sq)
    assert g.square_at(10.0, 0.0) is None


def test_rotated_board_round_trip():
    g = BoardGeometry(square_size=0.4, origin_x=1.0, origin_y=-2.0, yaw=math.pi / 2)
    x, y = g.square_center("a1")
    assert (x, y) == pytest.approx((1.0 + 1.4, -2.0 - 1.4))
    assert g.map_to_board(x, y) == pytest.approx(g.square_center_board("a1"))
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


@pytest.mark.parametrize("config", ["board_sim.yaml", "board_real.yaml"])
def test_shipped_configs_are_valid(config):
    g = BoardGeometry.from_dict(load_config(config)["board"])
    checks = g.validate()
    assert all(v >= 0.03 for v in checks.values())


def test_too_small_squares_are_rejected():
    with pytest.raises(ValueError) as err:
        BoardGeometry(square_size=0.5).validate()
    assert "square-side midpoint" in str(err.value)


def test_initial_models_unique():
    models = initial_square_models()
    assert len(models) == 32
    assert len(set(models.values())) == 32
    assert models["g1"] == "white_knight_g"
    assert models["e8"] == "black_king_e"
