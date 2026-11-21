import chess
import pytest

from robot_chess.move_inference import infer_move, occupancy
from robot_chess.move_planner import parse_move, plan_move


def _after(board, san):
    b = board.copy()
    b.push_san(san)
    return occupancy(b)


@pytest.mark.parametrize("fen,san", [
    (chess.STARTING_FEN, "e4"),
    (chess.STARTING_FEN, "Nf3"),
    ("rnbqkbnr/ppp1pppp/8/3p4/4P3/8/PPPP1PPP/RNBQKBNR w KQkq d6 0 2", "exd5"),
    ("r3k2r/8/8/8/8/8/8/R3K2R w KQkq - 0 1", "O-O"),
    ("r3k2r/8/8/8/8/8/8/R3K2R b KQkq - 0 1", "O-O-O"),
    ("rnbqkbnr/ppp1p1pp/8/3pPp2/8/8/PPPP1PPP/RNBQKBNR w KQkq f6 0 3", "exf6"),
    ("4k3/1P6/8/8/8/8/8/4K3 w - - 0 1", "b8=Q"),
])
def test_every_kind_of_move_is_recognised(fen, san):
    board = chess.Board(fen)
    result = infer_move(board, _after(board, san))
    assert result.status == "move"
    assert result.move == board.parse_san(san)


def test_no_change():
    board = chess.Board()
    assert infer_move(board, occupancy(board)).status == "no_change"


def test_half_finished_move_is_not_accepted():
    """Piece lifted but not yet put down (arm still moving): unexplained."""
    board = chess.Board()
    obs = list(occupancy(board))
    obs[chess.E2] = "."
    result = infer_move(board, "".join(obs))
    assert result.status == "unexplained"
    assert result.changed == [chess.E2]


def test_tolerance_accepts_one_misread_square():
    board = chess.Board()
    obs = list(_after(board, "d4"))
    obs[chess.H7] = "."                     # vision error far away
    assert infer_move(board, "".join(obs)).status == "unexplained"
    result = infer_move(board, "".join(obs), tolerance=1)
    assert result.status == "move" and result.move.uci() == "d2d4"


def test_capture_goes_to_graveyard_first():
    board = chess.Board("rnbqkbnr/ppp1pppp/8/3p4/4P3/8/PPPP1PPP/RNBQKBNR w KQkq d6 0 2")
    tasks, used = plan_move(board, parse_move(board, "exd5"), graveyard_used=2)
    assert [(t.from_label, t.to_label) for t in tasks] == [("d5", "graveyard-2"), ("e4", "d5")]
    assert tasks[0].piece_color == "black" and used == 3


def test_en_passant_and_castling_tasks():
    board = chess.Board("rnbqkbnr/ppp1p1pp/8/3pPp2/8/8/PPPP1PPP/RNBQKBNR w KQkq f6 0 3")
    tasks, _ = plan_move(board, parse_move(board, "exf6"))
    assert [(t.from_label, t.to_label) for t in tasks] == [("f5", "graveyard-0"), ("e5", "f6")]
    board = chess.Board("r3k2r/8/8/8/8/8/8/R3K2R w KQkq - 0 1")
    tasks, _ = plan_move(board, parse_move(board, "O-O-O"))
    assert [(t.from_label, t.to_label) for t in tasks] == [("e1", "c1"), ("a1", "d1")]


def test_promotion_flag_and_uci_default():
    board = chess.Board("4k3/P7/8/8/8/8/8/4K3 w - - 0 1")
    move = parse_move(board, "a7a8")
    assert move.promotion == chess.QUEEN
    tasks, _ = plan_move(board, move)
    assert tasks[0].promote_to == "queen"


def test_parse_errors():
    board = chess.Board()
    with pytest.raises(ValueError):
        parse_move(board, "e5")
    with pytest.raises(ValueError):
        parse_move(board, "")
