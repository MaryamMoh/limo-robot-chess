"""Decompose a chess move into physical pick-and-place tasks.

The referee owns the logical board (python-chess) and a second piece of
state: which physical piece (model) stands on which square, and how many
pieces of each colour already lie in the graveyard.  For a legal move this
module produces the ordered list of tasks the robot has to execute and the
updated physical state.

Order of the tasks:
  1. a captured piece (normal capture or en passant) goes to the graveyard
     first, so the destination square is free;
  2. the moving piece goes from its square to the destination square
     (with an optional promotion swap at the end);
  3. for castling, the rook is moved after the king.
"""
import copy

import chess

from robot_chess.pieces import (PIECE_TYPE_NAME, color_name,
                                promoted_model_name)


class PieceTask(object):
    """Plain python version of robot_chess_msgs/PieceTask."""

    __slots__ = ("model_name", "piece_color", "piece_type", "from_label",
                 "from_x", "from_y", "to_label", "to_x", "to_y",
                 "promote_to", "promoted_model_name")

    def __init__(self, **kwargs):
        for name in self.__slots__:
            setattr(self, name, kwargs.get(name, ""))

    def as_dict(self):
        return {name: getattr(self, name) for name in self.__slots__}

    def __repr__(self):
        return "PieceTask(%s %s -> %s%s)" % (
            self.model_name, self.from_label, self.to_label,
            (" =" + self.promote_to) if self.promote_to else "")


class PhysicalState(object):
    """Where every physical piece is."""

    def __init__(self, square_models, graveyard_counts=None, promotions=0):
        self.square_models = dict(square_models)
        self.graveyard_counts = dict(graveyard_counts or {"white": 0, "black": 0})
        self.promotions = int(promotions)

    def copy(self):
        return copy.deepcopy(self)

    def occupied_squares(self):
        return sorted(self.square_models.keys())


def _task(model, color, ptype, from_label, from_xy, to_label, to_xy):
    return PieceTask(model_name=model, piece_color=color, piece_type=ptype,
                     from_label=from_label, from_x=from_xy[0], from_y=from_xy[1],
                     to_label=to_label, to_x=to_xy[0], to_y=to_xy[1],
                     promote_to="", promoted_model_name="")


def _square_move_task(geometry, state, board, from_sq, to_sq):
    piece = board.piece_at(from_sq)
    fname, tname = chess.square_name(from_sq), chess.square_name(to_sq)
    model = state.square_models[fname]
    task = _task(model, color_name(piece.color),
                 PIECE_TYPE_NAME[piece.piece_type],
                 fname, geometry.square_center(from_sq),
                 tname, geometry.square_center(to_sq))
    del state.square_models[fname]
    state.square_models[tname] = model
    return task


def plan_move(board, move, state, geometry):
    """Return (tasks, new_state) for a legal move on `board`.

    `board` is the position *before* the move and is not modified.
    """
    if move not in board.legal_moves:
        raise ValueError("illegal move %s in %s" % (move.uci(), board.fen()))

    new_state = state.copy()
    tasks = []

    # 1. captured piece -> graveyard
    captured_square = None
    if board.is_en_passant(move):
        captured_square = chess.square(chess.square_file(move.to_square),
                                       chess.square_rank(move.from_square))
    elif board.is_capture(move):
        captured_square = move.to_square

    if captured_square is not None:
        victim = board.piece_at(captured_square)
        vcolor = color_name(victim.color)
        vname = chess.square_name(captured_square)
        slot = new_state.graveyard_counts[vcolor]
        tasks.append(_task(new_state.square_models[vname], vcolor,
                           PIECE_TYPE_NAME[victim.piece_type],
                           vname, geometry.square_center(captured_square),
                           "graveyard-%s-%d" % (vcolor, slot),
                           geometry.graveyard_slot(vcolor, slot)))
        del new_state.square_models[vname]
        new_state.graveyard_counts[vcolor] = slot + 1

    # 2. the moving piece
    main = _square_move_task(geometry, new_state, board,
                             move.from_square, move.to_square)
    if move.promotion:
        new_type = PIECE_TYPE_NAME[move.promotion]
        new_state.promotions += 1
        new_model = promoted_model_name(main.piece_color, new_type,
                                        new_state.promotions)
        main.promote_to = new_type
        main.promoted_model_name = new_model
        new_state.square_models[chess.square_name(move.to_square)] = new_model
    tasks.append(main)

    # 3. castling: the rook follows the king
    if board.is_castling(move):
        rank = chess.square_rank(move.from_square)
        if board.is_kingside_castling(move):
            rook_from, rook_to = chess.square(7, rank), chess.square(5, rank)
        else:
            rook_from, rook_to = chess.square(0, rank), chess.square(3, rank)
        tasks.append(_square_move_task(geometry, new_state, board,
                                       rook_from, rook_to))

    return tasks, new_state


def parse_move(board, text):
    """Parse SAN ("Nf3", "O-O", "e8=Q") or UCI ("g1f3", "e7e8q").

    A UCI pawn move to the last rank without a promotion letter is promoted
    to a queen.  Raises ValueError for unparsable or illegal moves.
    """
    text = text.strip()
    if not text:
        raise ValueError("empty move")
    # Accept 0-0 / 0-0-0 written with zeros.
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
