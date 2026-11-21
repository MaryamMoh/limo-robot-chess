# One pick-and-place job for the arm.  A chess move is decomposed into one
# or more of these (captures, castling and en passant need more than one).
string piece_color      # "white" | "black"
string piece_type       # "pawn" | "knight" | "bishop" | "rook" | "queen" | "king"
string from_label       # square name ("e2")
string to_label         # square name or "graveyard-<n>"
string promote_to       # "" or the new piece type (pawn promotion)
