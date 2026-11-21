"""Piece naming, colours and the colour code used for visual identification.

Every physical piece carries a coloured band at the height of the LIMO
camera.  The band colour encodes the piece type, the body colour (white /
black) encodes the side.  The same table is used to build the Gazebo
models and to classify camera images, so simulation and vision always agree.
"""
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

# Band colour (RGB 0..1) per piece type.  The hues are 60 degrees apart so
# they are easy to separate in HSV space.
BAND_RGB = {
    "rook": (0.85, 0.08, 0.08),     # red      hue   0
    "bishop": (0.85, 0.85, 0.08),   # yellow   hue  60
    "pawn": (0.08, 0.75, 0.08),     # green    hue 120
    "king": (0.08, 0.80, 0.80),     # cyan     hue 180
    "knight": (0.08, 0.08, 0.85),   # blue     hue 240
    "queen": (0.80, 0.08, 0.80),    # magenta  hue 300
}

# OpenCV hue (0..179) of every band colour.
BAND_HUE_CV = {
    "rook": 0,
    "bishop": 30,
    "pawn": 60,
    "king": 90,
    "knight": 120,
    "queen": 150,
}

BODY_RGB = {
    "white": (0.92, 0.92, 0.90),
    "black": (0.10, 0.10, 0.10),
}

# Physical dimensions [m].
PIECE_RADIUS = 0.09
BASE_HEIGHT = 0.04
BAND_BOTTOM = 0.12
BAND_TOP = 0.24
PIECE_HEIGHT = {
    "pawn": 0.32,
    "rook": 0.36,
    "knight": 0.38,
    "bishop": 0.40,
    "queen": 0.44,
    "king": 0.48,
}


def color_name(chess_color):
    """chess.WHITE / chess.BLACK -> "white" / "black"."""
    return "white" if chess_color == chess.WHITE else "black"


def opposite(color):
    return "black" if color == "white" else "white"


def initial_model_name(color, piece_type, square_name):
    """Stable name of a piece in the starting position, e.g. white_knight_g.

    The file of the start square makes the name unique (two knights, two
    rooks, eight pawns ...).
    """
    return "%s_%s_%s" % (color, piece_type, square_name[0])


def promoted_model_name(color, piece_type, index):
    return "%s_%s_promoted_%d" % (color, piece_type, index)


def initial_square_models(board=None):
    """Map square name -> model name for the standard start position."""
    board = board or chess.Board()
    result = {}
    for square, piece in board.piece_map().items():
        name = chess.square_name(square)
        result[name] = initial_model_name(
            color_name(piece.color), PIECE_TYPE_NAME[piece.piece_type], name)
    return result
