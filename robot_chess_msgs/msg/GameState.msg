# Published (latched) by chess_player on /chess/state.
string STATUS_CALIBRATING   = calibrating
string STATUS_THINKING      = thinking
string STATUS_MOVING        = moving
string STATUS_WAITING       = waiting_for_opponent
string STATUS_UNCLEAR       = opponent_move_unclear
string STATUS_ERROR         = error
string STATUS_GAME_OVER     = game_over

time stamp
string my_color             # "white" | "black"
string fen                  # current position
string turn                 # side to move
string status               # one of STATUS_*
string status_text          # human readable explanation
string last_move_san
string[] move_history_san
string result               # "1-0", "0-1", "1/2-1/2" or "*"
bool in_check
string observed_occupancy   # last camera reading, 64 chars a1..h8: '.', 'w', 'b'

# engine analysis of my last move
string engine
int32 score_cp              # centipawns from my point of view
int32 depth
string[] principal_variation
