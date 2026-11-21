# One physical "pick a piece here, put it there" job for a robot.
# A chess move is decomposed into one or more of these (captures,
# castling and en passant need more than one).

string model_name       # unique name of the piece (also its Gazebo model name)
string piece_color      # "white" | "black"
string piece_type       # "pawn" | "knight" | "bishop" | "rook" | "queen" | "king"

string from_label       # square name ("e2") or "graveyard-<n>"
float64 from_x          # pick-up position in the map frame [m]
float64 from_y
string to_label         # square name or "graveyard-<n>"
float64 to_x            # drop-off position in the map frame [m]
float64 to_y

# Promotion: after dropping, the piece is replaced by a new one.
string promote_to       # "" or the new piece type
string promoted_model_name
