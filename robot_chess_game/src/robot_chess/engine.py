"""Choosing the best move.

Two engines with the same interface `choose(board) -> SearchResult`:

InternalEngine (default, written for this project)
    * negamax search with alpha-beta pruning
    * iterative deepening under a time limit (the best move of the last
      completed depth is always available)
    * transposition table keyed by the Zobrist hash of the position
    * move ordering: transposition-table move, captures by MVV-LVA
      (most valuable victim, least valuable attacker), promotions, killers
    * quiescence search on captures at the leaves (avoids the horizon effect
      of stopping the search in the middle of an exchange)
    * evaluation: material + piece-square tables of the "Simplified
      Evaluation Function" (T. Michniewski), king table switched to the
      endgame version when queens are off the board

StockfishEngine
    the Stockfish UCI engine through python-chess (much stronger); falls
    back to the internal engine if the binary is not installed.
"""
import collections
import os
import shutil
import time

import chess
import chess.engine
import chess.polyglot

PIECE_VALUE = {chess.PAWN: 100, chess.KNIGHT: 320, chess.BISHOP: 330,
               chess.ROOK: 500, chess.QUEEN: 900, chess.KING: 0}

# Piece-square tables from white's point of view, index 0 = a8 ... 63 = h1
# (the way they are usually printed); converted below to python-chess
# square order (0 = a1).
_PST_TEXT = {
    chess.PAWN: """
         0   0   0   0   0   0   0   0
        50  50  50  50  50  50  50  50
        10  10  20  30  30  20  10  10
         5   5  10  25  25  10   5   5
         0   0   0  20  20   0   0   0
         5  -5 -10   0   0 -10  -5   5
         5  10  10 -20 -20  10  10   5
         0   0   0   0   0   0   0   0""",
    chess.KNIGHT: """
       -50 -40 -30 -30 -30 -30 -40 -50
       -40 -20   0   0   0   0 -20 -40
       -30   0  10  15  15  10   0 -30
       -30   5  15  20  20  15   5 -30
       -30   0  15  20  20  15   0 -30
       -30   5  10  15  15  10   5 -30
       -40 -20   0   5   5   0 -20 -40
       -50 -40 -30 -30 -30 -30 -40 -50""",
    chess.BISHOP: """
       -20 -10 -10 -10 -10 -10 -10 -20
       -10   0   0   0   0   0   0 -10
       -10   0   5  10  10   5   0 -10
       -10   5   5  10  10   5   5 -10
       -10   0  10  10  10  10   0 -10
       -10  10  10  10  10  10  10 -10
       -10   5   0   0   0   0   5 -10
       -20 -10 -10 -10 -10 -10 -10 -20""",
    chess.ROOK: """
         0   0   0   0   0   0   0   0
         5  10  10  10  10  10  10   5
        -5   0   0   0   0   0   0  -5
        -5   0   0   0   0   0   0  -5
        -5   0   0   0   0   0   0  -5
        -5   0   0   0   0   0   0  -5
        -5   0   0   0   0   0   0  -5
         0   0   0   5   5   0   0   0""",
    chess.QUEEN: """
       -20 -10 -10  -5  -5 -10 -10 -20
       -10   0   0   0   0   0   0 -10
       -10   0   5   5   5   5   0 -10
        -5   0   5   5   5   5   0  -5
         0   0   5   5   5   5   0  -5
       -10   5   5   5   5   5   0 -10
       -10   0   5   0   0   0   0 -10
       -20 -10 -10  -5  -5 -10 -10 -20""",
    chess.KING: """
       -30 -40 -40 -50 -50 -40 -40 -30
       -30 -40 -40 -50 -50 -40 -40 -30
       -30 -40 -40 -50 -50 -40 -40 -30
       -30 -40 -40 -50 -50 -40 -40 -30
       -20 -30 -30 -40 -40 -30 -30 -20
       -10 -20 -20 -20 -20 -20 -20 -10
        20  20   0   0   0   0  20  20
        20  30  10   0   0  10  30  20""",
}
_KING_END_TEXT = """
       -50 -40 -30 -20 -20 -30 -40 -50
       -30 -20 -10   0   0 -10 -20 -30
       -30 -10  20  30  30  20 -10 -30
       -30 -10  30  40  40  30 -10 -30
       -30 -10  30  40  40  30 -10 -30
       -30 -10  20  30  30  20 -10 -30
       -30 -30   0   0   0   0 -30 -30
       -50 -30 -30 -30 -30 -30 -30 -50"""


def _parse_table(text):
    values = [int(v) for v in text.split()]
    assert len(values) == 64
    # printed row 0 is rank 8 -> python-chess square = (7 - row) * 8 + col
    table = [0] * 64
    for i, v in enumerate(values):
        row, col = divmod(i, 8)
        table[(7 - row) * 8 + col] = v
    return table


PST = {pt: _parse_table(t) for pt, t in _PST_TEXT.items()}
KING_END = _parse_table(_KING_END_TEXT)

MATE = 100000
INF = 10 ** 9

SearchResult = collections.namedtuple("SearchResult", "move score depth nodes pv engine")


def evaluate(board):
    """Static evaluation in centipawns from the side to move's view."""
    endgame = not board.pieces(chess.QUEEN, chess.WHITE) and \
        not board.pieces(chess.QUEEN, chess.BLACK)
    score = 0
    for square, piece in board.piece_map().items():
        if piece.piece_type == chess.KING and endgame:
            table = KING_END
        else:
            table = PST[piece.piece_type]
        # tables are written for white; mirror the square for black
        idx = square if piece.color == chess.WHITE else chess.square_mirror(square)
        value = PIECE_VALUE[piece.piece_type] + table[idx]
        score += value if piece.color == chess.WHITE else -value
    return score if board.turn == chess.WHITE else -score


class _Timeout(Exception):
    pass


class InternalEngine(object):
    name = "internal"

    def __init__(self, time_limit=3.0, max_depth=5):
        self.time_limit = float(time_limit)
        self.max_depth = int(max_depth)
        self.tt = {}
        self.killers = collections.defaultdict(list)

    # ------------------------------------------------------------- ordering
    def _order(self, board, moves, tt_move, ply):
        def key(m):
            if m == tt_move:
                return -10 ** 6
            score = 0
            if board.is_capture(m):
                victim = board.piece_at(m.to_square)
                victim_value = PIECE_VALUE[victim.piece_type] if victim else PIECE_VALUE[chess.PAWN]
                attacker = board.piece_at(m.from_square)
                score -= 10 * victim_value - PIECE_VALUE[attacker.piece_type] + 10000
            if m.promotion:
                score -= PIECE_VALUE[m.promotion] + 5000
            if m in self.killers[ply]:
                score -= 3000
            return score
        return sorted(moves, key=key)

    # --------------------------------------------------------------- search
    def _check_time(self):
        self.nodes += 1
        if self.nodes & 1023 == 0 and time.time() > self.deadline:
            raise _Timeout()

    def _quiescence(self, board, alpha, beta):
        self._check_time()
        stand_pat = evaluate(board)
        if stand_pat >= beta:
            return beta
        alpha = max(alpha, stand_pat)
        captures = [m for m in board.legal_moves if board.is_capture(m) or m.promotion]
        for m in self._order(board, captures, None, 0):
            board.push(m)
            score = -self._quiescence(board, -beta, -alpha)
            board.pop()
            if score >= beta:
                return beta
            alpha = max(alpha, score)
        return alpha

    def _negamax(self, board, depth, alpha, beta, ply):
        self._check_time()
        if board.is_checkmate():
            return -MATE + ply                 # prefer the fastest mate
        if board.is_stalemate() or board.is_insufficient_material() or \
                board.is_repetition(2) or board.halfmove_clock >= 100:
            return 0
        if depth <= 0:
            return self._quiescence(board, alpha, beta)

        key = chess.polyglot.zobrist_hash(board)
        entry = self.tt.get(key)
        tt_move = None
        if entry is not None:
            e_depth, e_score, e_flag, tt_move = entry
            if e_depth >= depth:
                if e_flag == 0:
                    return e_score
                if e_flag < 0 and e_score <= alpha:
                    return e_score
                if e_flag > 0 and e_score >= beta:
                    return e_score

        original_alpha = alpha
        best_score, best_move = -INF, None
        for m in self._order(board, list(board.legal_moves), tt_move, ply):
            board.push(m)
            score = -self._negamax(board, depth - 1, -beta, -alpha, ply + 1)
            board.pop()
            if score > best_score:
                best_score, best_move = score, m
            alpha = max(alpha, score)
            if alpha >= beta:
                if not board.is_capture(m):
                    killers = self.killers[ply]
                    if m not in killers:
                        killers.insert(0, m)
                        del killers[2:]
                break

        flag = 0
        if best_score <= original_alpha:
            flag = -1                          # upper bound
        elif best_score >= beta:
            flag = 1                           # lower bound
        self.tt[key] = (depth, best_score, flag, best_move)
        return best_score

    def _principal_variation(self, board, length):
        pv, b = [], board.copy()
        for _ in range(length):
            entry = self.tt.get(chess.polyglot.zobrist_hash(b))
            if entry is None or entry[3] is None or entry[3] not in b.legal_moves:
                break
            pv.append(entry[3])
            b.push(entry[3])
        return pv

    def choose(self, board):
        moves = list(board.legal_moves)
        if not moves:
            return SearchResult(None, evaluate(board), 0, 0, [], self.name)
        self.deadline = time.time() + self.time_limit
        self.nodes = 0
        self.killers.clear()
        if len(self.tt) > 500000:
            self.tt.clear()
        best = SearchResult(self._order(board, moves, None, 0)[0], 0, 0, 0, [], self.name)
        work = board.copy()
        for depth in range(1, self.max_depth + 1):
            try:
                score = self._negamax(work, depth, -INF, INF, 0)
            except _Timeout:
                break
            pv = self._principal_variation(board, depth)
            if pv:
                best = SearchResult(pv[0], score, depth, self.nodes, pv, self.name)
            if abs(score) > MATE - 100:        # mate found, no need to go deeper
                break
        return best


class StockfishEngine(object):
    name = "stockfish"

    def __init__(self, time_limit=1.0, path=None, skill_level=None):
        self.time_limit = float(time_limit)
        self.path = path or shutil.which("stockfish") or "/usr/games/stockfish"
        self.skill_level = skill_level
        self.engine = None

    def _open(self):
        if self.engine is None:
            self.engine = chess.engine.SimpleEngine.popen_uci(self.path)
            if self.skill_level is not None:
                self.engine.configure({"Skill Level": int(self.skill_level)})

    def choose(self, board):
        self._open()
        info = self.engine.analyse(board, chess.engine.Limit(time=self.time_limit))
        pv = info.get("pv", [])
        score = info["score"].pov(board.turn).score(mate_score=MATE)
        return SearchResult(pv[0] if pv else None, score, info.get("depth", 0),
                            info.get("nodes", 0), pv, self.name)

    def close(self):
        if self.engine is not None:
            self.engine.quit()
            self.engine = None


def make_engine(kind="internal", time_limit=3.0, **kwargs):
    """'stockfish' if requested and installed, otherwise the internal engine."""
    if kind == "stockfish":
        path = kwargs.get("path") or shutil.which("stockfish")
        if path or os.path.exists("/usr/games/stockfish"):
            return StockfishEngine(time_limit, path, kwargs.get("skill_level"))
    return InternalEngine(time_limit, kwargs.get("max_depth", 5))
