"""Decompose one of *my* moves into pick-and-place tasks for the arm.

Order of the tasks:
  1. a captured opponent piece (normal capture or en passant) goes to the
     next free graveyard slot first, so the destination square is free;
  2. the moving piece goes from its square to the destination square;
     a promotion is flagged on this task (the piece must be exchanged);
  3. for castling, the rook is moved after the king.

Labels are square names ("e2") or "graveyard-<n>"; BoardGeometry turns them
into arm targets.
"""
import chess

from robot_chess.pieces import PIECE_TYPE_NAME, color_name


class PieceTask(object):
    """Plain python version of robot_chess_msgs/PieceTask."""

    __slots__ = ("piece_color", "piece_type", "from_label", "to_label", "promote_to")

    def __init__(self, piece_color, piece_type, from_label, to_label, promote_to=""):
        self.piece_color = piece_color
        self.piece_type = piece_type
        self.from_label = from_label
        self.to_label = to_label
        self.promote_to = promote_to

    def as_dict(self):
        return {name: getattr(self, name) for name in self.__slots__}

    def __repr__(self):
        return "PieceTask(%s %s %s -> %s%s)" % (
            self.piece_color, self.piece_type, self.from_label, self.to_label,
            (" =" + self.promote_to) if self.promote_to else "")


def _task(board, from_sq, to_label):
    piece = board.piece_at(from_sq)
    return PieceTask(color_name(piece.color), PIECE_TYPE_NAME[piece.piece_type],
                     chess.square_name(from_sq), to_label)


def plan_move(board, move, graveyard_used=0):
    """(tasks, new_graveyard_used) for a legal move on `board` (before the move)."""
    if move not in board.legal_moves:
        raise ValueError("illegal move %s in %s" % (move.uci(), board.fen()))
    tasks = []

    captured = None
    if board.is_en_passant(move):
        captured = chess.square(chess.square_file(move.to_square),
                                chess.square_rank(move.from_square))
    elif board.is_capture(move):
        captured = move.to_square
    if captured is not None:
        tasks.append(_task(board, captured, "graveyard-%d" % graveyard_used))
        graveyard_used += 1

    main = _task(board, move.from_square, chess.square_name(move.to_square))
    if move.promotion:
        main.promote_to = PIECE_TYPE_NAME[move.promotion]
    tasks.append(main)

    if board.is_castling(move):
        rank = chess.square_rank(move.from_square)
        if board.is_kingside_castling(move):
            rook_from, rook_to = chess.square(7, rank), chess.square(5, rank)
        else:
            rook_from, rook_to = chess.square(0, rank), chess.square(3, rank)
        tasks.append(_task(board, rook_from, chess.square_name(rook_to)))

    return tasks, graveyard_used


def parse_move(board, text):
    """Parse SAN ("Nf3", "O-O", "e8=Q") or UCI ("g1f3", "e7e8q")."""
    text = text.strip()
    if not text:
        raise ValueError("empty move")
    normalized = text.replace("0-0-0", "O-O-O").replace("0-0", "O-O")
    try:
        return board.parse_san(normalized)
    except ValueError:
        pass
    try:
        move = chess.Move.from_uci(text.lower())
    except ValueError:
        raise ValueError("cannot understand move '%s'" % text)
    if move.promotion is None:
        piece = board.piece_at(move.from_square)
        if (piece is not None and piece.piece_type == chess.PAWN
                and chess.square_rank(move.to_square) in (0, 7)):
            move = chess.Move(move.from_square, move.to_square, chess.QUEEN)
    if move not in board.legal_moves:
        raise ValueError("illegal move '%s'" % text)
    return move
