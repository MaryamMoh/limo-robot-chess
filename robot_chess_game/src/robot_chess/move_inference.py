"""Recognise the opponent's move from what the camera sees.

The camera only tells, for every square, whether it is empty, holds a white
piece or holds a black piece ("occupancy").  Since we know the position
before the opponent moved, the move is the legal move whose resulting
occupancy matches the observation:

    for every legal move m:
        mismatch(m) = number of squares where occupancy(board after m)
                      differs from the observed occupancy

The move with zero mismatches is accepted.  This handles normal moves,
captures (a square changes colour), castling (four squares change),
en passant (three squares change) and promotion (indistinguishable from the
camera: a queen is assumed, which is what players choose in > 95 % of the
cases; the chess_player node offers an override service for the rest).
"""
import chess

EMPTY, WHITE, BLACK = ".", "w", "b"


def occupancy(board):
    """64-character string indexed by python-chess square (a1 = 0)."""
    out = []
    for sq in chess.SQUARES:
        piece = board.piece_at(sq)
        out.append(EMPTY if piece is None else (WHITE if piece.color else BLACK))
    return "".join(out)


def differences(a, b):
    """Squares where two occupancy strings differ."""
    return [sq for sq in chess.SQUARES if a[sq] != b[sq]]


def candidate_moves(board, observed):
    """[(mismatch, move)] for all legal moves, best first.  Promotions to
    pieces other than a queen are skipped (they look identical)."""
    result = []
    for move in board.legal_moves:
        if move.promotion and move.promotion != chess.QUEEN:
            continue
        board.push(move)
        mismatch = len(differences(occupancy(board), observed))
        board.pop()
        result.append((mismatch, move))
    result.sort(key=lambda t: (t[0], t[1].uci()))
    return result


class InferenceResult(object):
    """status: 'no_change', 'move', 'unexplained'."""

    def __init__(self, status, move=None, changed=None, best=None):
        self.status = status
        self.move = move
        self.changed = changed or []
        self.best = best or []

    def __repr__(self):
        return "InferenceResult(%s, %s, changed=%s)" % (
            self.status, self.move, [chess.square_name(s) for s in self.changed])


def infer_move(board, observed, tolerance=0):
    """Explain the observed occupancy by one legal move of the side to move.

    tolerance: number of squares that may still disagree (e.g. a known
    unreliable square); 0 means an exact match is required.  A unique best
    candidate is needed when tolerance > 0.
    """
    changed = differences(occupancy(board), observed)
    if not changed:
        return InferenceResult("no_change")
    candidates = candidate_moves(board, observed)
    if not candidates:
        return InferenceResult("unexplained", changed=changed)
    best_mismatch, best_move = candidates[0]
    unique = len(candidates) == 1 or candidates[1][0] > best_mismatch
    if best_mismatch == 0 or (best_mismatch <= tolerance and unique):
        return InferenceResult("move", best_move, changed, candidates[:3])
    return InferenceResult("unexplained", changed=changed, best=candidates[:3])
