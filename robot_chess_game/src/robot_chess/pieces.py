"""Piece names, colours and dimensions."""
import chess

COLORS = ("white", "black")
TYPES = ("pawn", "knight", "bishop", "rook", "queen", "king")

PIECE_TYPE_NAME = {
    chess.PAWN: "pawn",
    chess.KNIGHT: "knight",
    chess.BISHOP: "bishop",
    chess.ROOK: "rook",
    chess.QUEEN: "queen",
    chess.KING: "king",
}
PIECE_TYPE_FROM_NAME = {v: k for k, v in PIECE_TYPE_NAME.items()}

BODY_RGB = {
    "white": (0.93, 0.92, 0.86),
    "black": (0.12, 0.10, 0.10),
}

# Simulated pieces (a small travel set) [m].
PIECE_RADIUS = 0.0065
PIECE_HEIGHT = {
    "pawn": 0.018,
    "rook": 0.020,
    "knight": 0.022,
    "bishop": 0.024,
    "queen": 0.027,
    "king": 0.030,
}


def color_name(chess_color):
    """chess.WHITE / chess.BLACK -> "white" / "black"."""
    return "white" if chess_color == chess.WHITE else "black"


def chess_color(name):
    return chess.WHITE if name == "white" else chess.BLACK


def opposite(color):
    return "black" if color == "white" else "white"


def initial_model_name(color, piece_type, square_name):
    """Stable Gazebo name of a piece of the start position, e.g. white_knight_g."""
    return "%s_%s_%s" % (color, piece_type, square_name[0])


def initial_square_models(board=None):
    """Map square name -> model name for the standard start position."""
    board = board or chess.Board()
    result = {}
    for square, piece in board.piece_map().items():
        name = chess.square_name(square)
        result[name] = initial_model_name(
            color_name(piece.color), PIECE_TYPE_NAME[piece.piece_type], name)
    return result
