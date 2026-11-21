import chess
import pytest

from robot_chess.engine import InternalEngine, evaluate, make_engine


def test_start_position_is_balanced():
    assert evaluate(chess.Board()) == 0


def test_evaluation_is_side_relative():
    board = chess.Board("4k3/8/8/8/8/8/8/3QK3 w - - 0 1")
    assert evaluate(board) > 800
    board.turn = chess.BLACK
    assert evaluate(board) < -800


@pytest.mark.parametrize("fen,best", [
    # mate in one: back rank
    ("6k1/5ppp/8/8/8/8/8/R5K1 w - - 0 1", "a1a8"),
    # mate in one for black
    ("r5k1/8/8/8/8/8/5PPP/6K1 b - - 0 1", "a8a1"),
    # free queen
    ("4k3/8/8/3q4/8/8/8/3RK3 w - - 0 1", "d1d5"),
])
def test_finds_tactics(fen, best):
    result = InternalEngine(time_limit=5.0, max_depth=3).choose(chess.Board(fen))
    assert result.move.uci() == best


def test_avoids_losing_the_queen():
    # Qxd7?? loses the queen to the rook on d8 (quiescence sees the recapture)
    board = chess.Board("3r2k1/3p4/8/8/8/8/3Q4/6K1 w - - 0 1")
    result = InternalEngine(time_limit=5.0, max_depth=2).choose(board)
    assert result.move.uci() != "d2d7"


def test_returns_legal_move_under_time_pressure():
    board = chess.Board()
    result = InternalEngine(time_limit=0.2).choose(board)
    assert result.move in board.legal_moves


def test_no_move_when_game_over():
    board = chess.Board("7k/5QQ1/8/8/8/8/8/6K1 b - - 0 1")   # checkmated
    assert board.is_game_over()
    assert InternalEngine(time_limit=0.5).choose(board).move is None


def test_stockfish_falls_back_when_missing(monkeypatch):
    import shutil
    monkeypatch.setattr(shutil, "which", lambda _: None)
    monkeypatch.setattr("os.path.exists", lambda _: False)
    assert isinstance(make_engine("stockfish"), InternalEngine)
