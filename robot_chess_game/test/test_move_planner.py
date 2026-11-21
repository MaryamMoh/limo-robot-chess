import chess
import pytest

from robot_chess.board_geometry import BoardGeometry
from robot_chess.move_planner import PhysicalState, parse_move, plan_move
from robot_chess.pieces import initial_square_models

G = BoardGeometry()


def _state():
    return PhysicalState(initial_square_models())


def _play(board, state, *moves):
    for text in moves:
        move = parse_move(board, text)
        _, state = plan_move(board, move, state, G)
        board.push(move)
    return state


def test_simple_move():
    board, state = chess.Board(), _state()
    tasks, new = plan_move(board, parse_move(board, "Nf3"), state, G)
    assert len(tasks) == 1
    t = tasks[0]
    assert (t.model_name, t.from_label, t.to_label) == ("white_knight_g", "g1", "f3")
    assert (t.to_x, t.to_y) == pytest.approx(G.square_center("f3"))
    assert "g1" not in new.square_models and new.square_models["f3"] == "white_knight_g"
    # original state untouched
    assert state.square_models["g1"] == "white_knight_g"


def test_capture_goes_to_graveyard_first():
    board, state = chess.Board(), _state()
    state = _play(board, state, "e4", "d5")
    tasks, new = plan_move(board, parse_move(board, "exd5"), state, G)
    assert [t.model_name for t in tasks] == ["black_pawn_d", "white_pawn_e"]
    assert tasks[0].to_label == "graveyard-black-0"
    assert (tasks[0].to_x, tasks[0].to_y) == pytest.approx(G.graveyard_slot("black", 0))
    assert new.graveyard_counts == {"white": 0, "black": 1}
    assert new.square_models["d5"] == "white_pawn_e"


def test_en_passant_removes_pawn_from_side_square():
    board, state = chess.Board(), _state()
    state = _play(board, state, "e4", "a6", "e5", "d5")
    tasks, new = plan_move(board, parse_move(board, "exd6"), state, G)
    assert tasks[0].from_label == "d5" and tasks[0].model_name == "black_pawn_d"
    assert tasks[1].from_label == "e5" and tasks[1].to_label == "d6"
    assert "d5" not in new.square_models


@pytest.mark.parametrize("san,king_to,rook_from,rook_to", [
    ("O-O", "g1", "h1", "f1"),
    ("O-O-O", "c1", "a1", "d1"),
])
def test_castling_moves_king_then_rook(san, king_to, rook_from, rook_to):
    board = chess.Board("r3k2r/8/8/8/8/8/8/R3K2R w KQkq - 0 1")
    state = PhysicalState(initial_square_models(board))
    tasks, new = plan_move(board, parse_move(board, san), state, G)
    assert [(t.from_label, t.to_label) for t in tasks] == [
        ("e1", king_to), (rook_from, rook_to)]
    assert new.square_models[rook_to] == state.square_models[rook_from]


def test_promotion_with_capture():
    board = chess.Board("1n2k3/P7/8/8/8/8/8/4K3 w - - 0 1")
    state = PhysicalState(initial_square_models(board))
    tasks, new = plan_move(board, parse_move(board, "axb8=N"), state, G)
    assert tasks[0].to_label == "graveyard-black-0"
    assert tasks[1].promote_to == "knight"
    assert tasks[1].promoted_model_name == "white_knight_promoted_1"
    assert new.square_models["b8"] == "white_knight_promoted_1"
    assert new.promotions == 1


def test_uci_promotion_defaults_to_queen():
    board = chess.Board("4k3/P7/8/8/8/8/8/4K3 w - - 0 1")
    assert parse_move(board, "a7a8") == chess.Move.from_uci("a7a8q")


def test_parse_variants_and_errors():
    board = chess.Board()
    assert parse_move(board, "g1f3") == parse_move(board, "Nf3")
    with pytest.raises(ValueError):
        parse_move(board, "e5")
    with pytest.raises(ValueError):
        parse_move(board, "hello")
    with pytest.raises(ValueError):
        parse_move(board, "")


def test_full_game_keeps_physical_state_consistent():
    """Scholar's mate style game with captures; physical and logical boards agree."""
    board, state = chess.Board(), _state()
    state = _play(board, state, "e4", "e5", "Qh5", "Nc6", "Bc4", "Nf6", "Qxf7#")
    assert board.is_checkmate()
    assert set(state.square_models) == {chess.square_name(s) for s in board.piece_map()}
    assert state.graveyard_counts == {"white": 0, "black": 1}
