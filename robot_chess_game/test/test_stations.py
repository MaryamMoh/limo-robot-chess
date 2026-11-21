import chess

from robot_chess.board_geometry import BoardGeometry
from robot_chess.kinematics import MyCobotKinematics
from robot_chess.stations import choose_station, coverage


def test_deep_board_needs_stations_and_they_cover_it():
    """A 3 cm board (24 x 24 cm) is not reachable from one place, but with
    the base driving forward to two more stations every square is."""
    g = BoardGeometry(square_size=0.03, board_x=0.21, graveyard_x=0.135,
                      graveyard_y=-0.105, graveyard_dx=0.025, graveyard_dy=0.03)
    k = MyCobotKinematics()
    squares = [chess.square_name(s) for s in chess.SQUARES]
    fixed = coverage(g, k, squares, [0.0])
    assert any(not v for v in fixed.values())
    both = coverage(g, k, squares, [0.0, 0.06, 0.12])
    assert all(both.values())
    assert choose_station(g, k, "e8", [0.0, 0.06, 0.12]) > 0.0
    assert choose_station(g, k, "e1", [0.0, 0.06, 0.12]) == 0.0
    assert g.base_offset == 0.0
