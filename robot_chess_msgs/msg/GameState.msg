# Published (latched) by the referee on /chess/state.
string STATUS_WAITING_FOR_MOVE = waiting_for_move
string STATUS_ROBOT_MOVING     = robot_moving
string STATUS_ROBOT_ERROR      = robot_error
string STATUS_GAME_OVER        = game_over

time stamp
string fen                  # full position (python-chess FEN)
string turn                 # "white" | "black"
string status               # one of STATUS_*
string status_text          # human readable explanation
string last_move_san
string last_move_uci
string[] move_history_san
string result               # "1-0", "0-1", "1/2-1/2" or "*"
bool in_check
string draw_offered_by      # "" | "white" | "black"
string[] robot_colors       # colours that are played by a physical robot
