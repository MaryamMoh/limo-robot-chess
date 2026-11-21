# System Design: Chess-Playing LIMO + myCobot

## 1. Setting

Two robots play chess against each other on a real board. Each is a LIMO with
a myCobot 280 arm, programmed by a different person. This repository is one
of the players. Nothing is shared between the two robots except the board,
so my robot must:

1. know the rules and the state of the game;
2. see what the opponent did;
3. decide on a good move;
4. physically make that move without disturbing the other pieces.

It is developed and tested in Gazebo, where a stand-in plays the opponent's
side, and then deployed on the real robot in the laboratory.

## 2. Hardware

| Item | Used for |
|---|---|
| AgileX LIMO | mobile base carrying the arm; optionally drives between base stations |
| myCobot 280, 6 DOF | moves the pieces; kinematics from AgileX's `limo_cobot_sim` model |
| parallel gripper | grasps pieces from above |
| wide-angle wrist camera (USB, 640×480) | reads the board |
| Jetson (LIMO) | runs all nodes; `pymycobot` talks to the arm over `/dev/ttyTHS1` |

## 3. Architecture

![architecture](architecture.svg)

| Node | Role |
|---|---|
| `chess_player` | game state, engine, turn logic, opponent move recognition, verification |
| `board_vision` | `observe_board` service: board occupancy from the wrist camera |
| `arm_executor` | `execute_move` action: pick and place with the arm; observe pose; base stations |
| `mycobot_driver` (real) | real arm behind `FollowJointTrajectory` interfaces + `joint_states` |
| ros_control controllers (sim) | the same interfaces in Gazebo |
| `board_visualizer`, `chess_monitor` | RViz markers and a terminal view of the game |
| `chess_sim` (sim) | spawns pieces, virtual grasp, ground-truth occupancy |
| `opponent_simulator` (sim) | plays the other side by looking only at the board |

The core algorithms are in a ROS-independent Python package
(`robot_chess_game/src/robot_chess`), so they can be unit-tested without ROS:

| Module | Contents |
|---|---|
| `kinematics.py` | forward kinematics from the URDF joint table; damped-least-squares IK |
| `engine.py` | own alpha-beta engine; Stockfish wrapper |
| `board_vision.py` | projection, colour features, self-calibrating classifier |
| `move_inference.py` | which legal move explains an observed board |
| `move_planner.py` | my move → ordered pick-and-place tasks |
| `board_geometry.py` | board pose, squares, graveyard, observe pose |
| `stations.py` | base stations for boards deeper than the arm's reach |

## 4. Game logic (`chess_player`)

```
calibrate  → [my turn?] ── yes → think → arm → verify → push move ─┐
                  │                                                 │
                  no → read board until it changes and is stable ──┤
                        → infer move → push move ───────────────────┘
```

- **Rules.** `python-chess` holds the position and provides legal moves,
  check, mate, stalemate, insufficient material, the 50/75-move rules and
  repetition.
- **Waiting for the opponent.**
  - The board is read every `observe_period`. A changed reading is accepted
    only after `stable_observations` identical readings in a row, so the other
    arm has finished and moved away.
  - If no legal move explains the change, the status becomes
    *opponent_move_unclear* and the robot keeps watching.
  - The `override_opponent_move` service resolves cases the camera cannot,
    such as an under-promotion.
- **Verifying my move.** After its own move, the robot re-reads the board and
  compares it with the expected occupancy. A difference is reported.
- **Output.** `GameState` carries the FEN, history, status, the last camera
  reading, and the engine's evaluation and principal variation.

## 5. Engine (`engine.py`)

The internal engine is written for this project.

### Search

1. **Negamax with alpha-beta pruning.** Scores are always from the side to
   move: `score = −search(child)`. Branches that cannot change the result
   are cut (`alpha ≥ beta`).
2. **Iterative deepening, depth 1, 2, 3, …, until the time limit**
   (`think_time`). The best move of the last completed depth is used, so
   there is always an answer.
3. **Transposition table.** Positions are keyed by Zobrist hash and store
   depth, score, bound type and best move. It avoids re-searching
   transpositions and provides the first move to try.
4. **Move ordering.** First the table move, then captures sorted by MVV-LVA
   (most valuable victim, least valuable attacker), then promotions, then
   killer moves (quiet moves that caused cut-offs at the same depth). Good
   ordering is what makes alpha-beta efficient.
5. **Quiescence search.** At depth 0 the search continues with captures only,
   so a position is never judged in the middle of an exchange (the horizon
   effect).
6. **Terminal positions.** Mate is scored −100000 + ply, so the engine prefers
   faster mates. Stalemate, insufficient material, repetition and the 50-move
   rule score 0.

### Evaluation

Material plus the piece-square tables of the "Simplified Evaluation Function"
(T. Michniewski). The king switches to the endgame table once queens are off
the board.

### Strength and tests

In Python it searches about 3–5 plies in a few seconds. That is enough to
find mates in one, win material, and avoid hanging the queen; the unit tests
cover all three.

`engine: stockfish` in the config uses Stockfish through UCI and falls back
to the internal engine if Stockfish is not installed.

## 6. Seeing the board (`board_vision.py`)

1. **Camera pose.** The camera is on the wrist, so its pose comes from the arm:
   `joint_states` → `robot_state_publisher` → TF `base_footprint →
   wrist_camera_optical`. The board pose is known in `base_footprint` from the
   config. Together they give the transform from the board to the camera.
2. **Projection.** Each square centre is projected with the pinhole model
   `u = fx·X/Z + cx`, `v = fy·Y/Z + cy`, using the intrinsics from
   `camera_info`. The point used is 1.2 cm above the board (the piece top), so
   pieces seen at an angle are sampled where they really appear.
3. **Features.** The mean CIE Lab colour of a small window around each
   projected point, averaged over 3 frames.
4. **Classification.** Nearest centroid over four classes: empty light square,
   empty dark square, white piece, black piece.
   - **Self-calibration:** the centroids are learnt from the start position,
     whose occupancy is known. The robot therefore adapts to any board, piece
     set and lighting without hand-tuned thresholds.
   - The confidence of a square is the relative margin to the nearest class
     with a different occupancy.
5. **Validation.** The unit tests render camera images of positions and check
   that move sequences (including captures and castling) are recognised from
   the images.

## 7. Recognising the opponent's move (`move_inference.py`)

The camera reports occupancy only (`.`, `w`, `b` per square). For every legal
move of the opponent, the occupancy after that move is compared with the
observation; the move with zero differing squares is the one played. This
handles:

- normal moves and captures (a square changes colour);
- castling (four squares change);
- en passant (three squares change);
- promotion (assumed to a queen).

A half-finished move (piece lifted, not yet placed) matches no move and is
reported as unexplained. A tolerance of one misread square can be allowed when
the best candidate is unique.

## 8. Making a move

### 8.1 Tasks (`move_planner.py`)

| Move | Tasks |
|---|---|
| quiet | piece from → to |
| capture | victim → next graveyard slot, then piece from → to |
| en passant | victim (beside the pawn) → graveyard, then pawn |
| castling | king, then rook |
| promotion | pawn from → to, then exchange the piece (sim: model swap; real: operator) |

### 8.2 Kinematics (`kinematics.py`)

**Forward kinematics.** Built from the joint table of AgileX's LIMO + myCobot
URDF: an origin (xyz, rpy) per joint, with each joint rotating about its z
axis. Our URDF uses the same table, and a unit test checks the upright zero
pose against it.

**Inverse kinematics** uses damped least squares:

```
Δq = Jᵀ (J Jᵀ + λ² I)⁻¹ e
```

The error `e` has these parts:
- 3 terms for the TCP position;
- 2 for the tool axis pointing straight down;
- optionally 1 for the tool yaw, used at the observe pose to point the camera
  forward.

The Jacobian `J` is numerical. Steps are clipped to 0.3 rad and joints to
±165° (myCobot limits). Several seeds are tried, and the solution nearest to
the previous one is used, which gives smooth paths.

### 8.3 Motions (`arm_executor.py`)

Each task becomes:

```
above(from) → down → close → up → above(to) → down → open → up
```

- **Straight paths.** Each straight tool segment is sampled every 1 cm and
  solved with IK seeded by the previous solution. The whole segment is sent as
  one joint trajectory, timed so that no joint exceeds the speed limit.
- **Heights.** 1.2 cm above the board to grasp, 6 cm to travel (the fingers
  clear the tallest piece by 2 cm).
- **After the move.** The arm returns to the observe pose so the camera can
  verify the result.

The gripper and arm are driven through `FollowJointTrajectory`:
- in Gazebo, by ros_control position controllers;
- on the real arm, by `mycobot_driver.py`, which streams `send_radians` and
  maps the finger opening to `set_gripper_value`.

### 8.4 Reach and base stations

With the tool pointing down, the myCobot 280 reaches about 7–27 cm from its
axis, and the LIMO body extends 16 cm forward. This gives two options.

**Fixed base (simulation default).**
- The board is a 16 cm travel board on a table whose top (22 cm) overhangs
  the bumper. The near rank is 10 cm from the arm axis.
- The graveyard (3 × 5 slots) is beside the h-file.
- The observe pose holds the tool 13 cm above the board, 4 cm towards the
  robot. The camera, mounted 4 cm ahead of the tool and tilted 10° away from
  the fingers, then sees all 64 squares.
- `check_reach.py` and a unit test verify the whole layout.

**Base stations.**
- For deeper boards the LIMO drives straight forward or back under the table
  to configured offsets.
- `stations.choose_station` picks, for each square, the nearest station from
  which grasp and travel poses are reachable.
- The base is moved with `cmd_vel` and odometry feedback, always with the arm
  lifted. It returns home before the camera reads the board.
- A unit test shows a 24 cm board needs three stations.

## 9. Simulation test bench

- **World.** `generate_world.py` builds the world from the player config, so
  the simulated board is exactly where the player expects it: a table with
  legs outside the robot's footprint, the board, the graveyard pad and the
  opponent's capture area.
- **Pieces.** 32 static models, 1.3 cm in diameter and 1.8–3 cm tall.
- **Virtual grasp.** Grasping such small objects with simulated fingers is
  unreliable in ODE. `chess_sim` therefore attaches the piece nearest to the
  TCP, and only if it is within 1.5 cm of it. The arm must still position the
  gripper correctly, or the grasp fails.
- **Opponent stand-in.** `opponent_simulator.py` behaves like the other team's
  robot:
  - it only looks at the board (ground truth) and recognises my moves with
    the same move inference;
  - it waits until the board has been stable for 3 s;
  - it replies with the internal engine (or random moves) and moves its
    pieces one after the other.
- **Noise.** The wrist camera has Gaussian image noise (σ = 0.007).

## 10. Tests

- **Unit tests** (`robot_chess_game/test`):
  - kinematics: zero pose, IK round trips, unreachable targets, smooth seeds;
  - layout: reachability of the whole simulation layout, base stations;
  - engine: tactics and time limits;
  - move recognition: every move type, half-finished moves, tolerance;
  - vision: rendered images, calibration, out-of-view detection;
  - move planning.
- **End-to-end** (`smoke_test.py`, run in CI in a ROS Noetic container):
  - six half-moves between my player and the stand-in in headless Gazebo;
  - it checks that the player had no errors and that both robots agree on
    every move;
  - it checks that the Gazebo pieces match the player's position.

## 11. Limitations and next steps

- The camera sees occupancy, not piece types, so opponent under-promotions need
  the override service.
- The real-arm path (`mycobot_driver.py`, camera mount, `tcp_offset`) must be
  checked on the laboratory robot. The values in `player_real.yaml` and in the
  URDF's `wrist_camera_joint` are where to adapt it.
- A failed arm move is reported but not undone automatically.
- A fixed overhead camera would also work: add its static TF and remap
  `board_vision`'s image topics. The vision code only needs the camera pose.
