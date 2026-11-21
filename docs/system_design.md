# System Design: LIMO Robot Chess

## 1. Goal

Two AgileX LIMO PRO robots play a game of chess on a floor chessboard. Each
robot belongs to one human player:

- the human chooses the move;
- the robot executes it physically, fully autonomously: it plans a route,
  drives, localises, verifies the piece with its camera, and carries it.

The system is developed and tested in Gazebo/RViz with two robots, and deployed
on a single real LIMO PRO in the laboratory.

## 2. Hardware and software platform

| Item | Used for |
|---|---|
| AgileX LIMO PRO, four-wheel differential mode | mobile base, turning in place, ≤ 0.3 m/s |
| NVIDIA Orin Nano | runs the robot's nodes |
| EAI T-mini Pro LiDAR (`/scan`) | AMCL localisation, safety stop |
| Orbbec Dabai depth camera (colour stream) | piece verification |
| IMU (`/imu`) | yaw rate for the EKF |
| Wheel odometry (`/odom`, `limo_base`) | EKF input |
| Ubuntu 20.04, ROS Noetic, Gazebo 11 | as on the LIMO PRO image |

The simulated robot (`robot_chess_description/urdf/limo_pro.urdf.xacro`)
reuses the geometry, masses, sensor placement and meshes of AgileX's
`limo_description`. Every frame and topic is prefixed with the robot name so
several robots can share one Gazebo world. All sensors are noisy:

- LiDAR: σ = 1.5 cm
- camera: σ = 0.01
- IMU: σ = 0.005

The chassis is moved by Gazebo's kinematic planar drive. The upstream
skid-steer plugin could not turn the four-wheel model in place: it measured
0.05 rad for a 1 rad command. Realistic motion errors are added by the
odometry noise model (4.4).

## 3. Architecture

![architecture](architecture.svg)

| Node | Package | Role |
|---|---|---|
| `chess_referee` | game | chess rules, turn order, move decomposition, dispatch to robots, game state |
| `player_console` (×2) | game | one terminal per human player |
| `board_visualizer` | game | logical board as RViz markers |
| `piece_mover` (per robot) | game | `execute_move` action server: route planning, driving, pick/place sequence |
| `piece_vision` (per robot) | game | `verify_piece` service: camera-based piece identification |
| `chess_carrier` | gazebo / game | sim: spawns pieces and carries them; real: asks an assistant |
| `noisy_odometry` (per robot, sim) | navigation | odometry motion model |
| `ekf` (per robot) | navigation | `robot_localization`: odometry + IMU fusion |
| `amcl` (per robot) | navigation | LiDAR localisation on the known map |
| `map_server` | navigation | known map of the arena / laboratory |

### Main interfaces (`robot_chess_msgs`)

- `SubmitMove.srv`: console → referee (`color`, `move` | `resign` | `draw`).
- `GameState.msg`: latched on `/chess/state` (FEN, turn, status, history, result).
- `ExecuteMove.action`: referee → robot, a list of `PieceTask`s, with feedback
  per stage (`approach`, `verify`, `pick`, `transport`, `place`, `return`).
- `PieceTask.msg`: one pick-and-place job (model, colour, type, from/to label
  and coordinates, optional promotion).
- `VerifyPiece.srv`: robot → camera.
- `AttachPiece` / `ReleasePiece` / `ReplacePiece`: robot → carrier.

### TF tree (per robot, simulation)

```
map ──amcl──> white/odom ──ekf──> white/base_footprint ──urdf──> white/base_link
                                                                  ├─ white/laser_link
                                                                  ├─ white/depth_camera_link ─ white/depth_link
                                                                  ├─ white/imu_link
                                                                  └─ white/carrier_plate, wheels
```

## 4. Modules

### 4.1 Chess logic and move decomposition

`python-chess` holds the logical position and decides legality, check, mate,
stalemate, insufficient material and the 75-move and fivefold-repetition rules.
Threefold repetition and the 50-move rule are applied when a player types `draw`.

Next to the logical board, the referee keeps a **physical state**: which model
stands on which square, and how many pieces lie in each graveyard.
`move_planner.plan_move()` turns a legal move into ordered tasks:

| Move | Tasks |
|---|---|
| quiet move | piece `from → to` |
| capture | victim `to → graveyard`, then piece `from → to` |
| en passant | victim (beside the pawn) `→ graveyard`, then pawn |
| castling | king `e → g/c`, then rook `h → f` / `a → d` |
| promotion | (capture), pawn `from → to`, then swap model for the new piece |

The referee commits the move only after the robot reports success. Physical
and logical state therefore never diverge silently.

### 4.2 Board geometry and road network

The board frame has its origin at the board centre, x pointing a→h and y
pointing 1→8. It is placed in the map frame by `(origin_x, origin_y, yaw)`,
so the same code serves the simulated arena and a taped laboratory board.

Pieces only ever stand on square centres. The lines *between* the squares are
therefore always free lanes:

- simulation: 0.8 m squares and 0.18 m pieces give 0.62 m lanes;
- the LIMO is 0.22 m wide.

`road_network.RoadNetwork` builds a graph of these lanes:

```
        home (black)
             |
    +--------+--------+   ring road, 0.5 m outside the board
    |  +--+--+--+--+  |
    |  |  |  |  |  |  |   lanes between the squares
  G-+--+--+--+--+--+--+-G graveyard spurs (one per slot)
    |  +--+--+--+--+  |
    +--------+--------+
             |
        home (white)
```

- **Nodes:** lane intersections, square-side midpoints, ring nodes and spur ends.
- **Approach points:** to handle a piece, the robot stands on one of the four
  midpoints of the square's sides and faces the centre. That is 0.4 m from
  the piece, and the camera sees it from 0.3 m. A graveyard slot has one
  approach point, between the ring road and the slot.
- **`BoardGeometry.validate()`** rejects a layout unless all of these hold:
  - the robot's circumscribed circle (r = 0.194 m) fits between the two pieces
    next to a midpoint, so turning there is always collision free;
  - the robot fits along a lane;
  - the ring road and graveyard spurs have room to turn.
- **Unit tests:** they check every edge of the graph against every possible
  piece position on the board and in the graveyards.

### 4.3 Route planning and driving (autonomous, noisy motion)

1. **Planning:** the robot's pose is projected onto the closest road segment.
   Multi-source Dijkstra from both ends of that segment finds the shortest route
   to the nearest of the target's approach points. The route is simplified to
   its corners, so it is a few axis-parallel straight segments. It is published
   on `<robot>/route` for RViz.
2. **Driving:** at each corner the robot turns in place towards the next corner,
   then drives straight.
   - Speed: v ≤ 0.3 m/s, slowing down near the corner.
   - Steering: proportional heading control, ω = 2.5·e.
   - A segment is finished when the remaining distance *along the segment* is
     below 3 cm, so lateral noise never makes the robot circle a corner.
3. **Final orientation:** at the approach point the robot turns to face the
   piece (tolerance 0.03 rad).
4. **Safety:** if the LiDAR sees anything closer than 0.25 m in a ±20° cone ahead
   (for example a person, or the other robot), the robot stops and waits. After
   60 s it gives up and reports an error.

On a structured chessboard this is more reliable than a general costmap
planner. In the first version, `move_base` with a DWA local planner could not
pass between the pieces once the 5 cm costmap discretisation and the inflation
were applied.

### 4.4 Noisy motion model

Gazebo's odometry is perfect. `noisy_odometry.py` corrupts each increment
between two ground-truth poses following the odometry motion model of Thrun
et al. (*Probabilistic Robotics*, §5.4). The increment is expressed as a
signed forward translation and a rotation in the robot frame, which is what
wheel encoders measure:

```
var(rot)   = a1·|rot|   + a2·|trans|        a1 = 0.002, a2 = 0.001
var(trans) = a3·|trans| + a4·|rot|          a3 = 0.002, a4 = 0.00001
```

The noisy increments are integrated with the midpoint rule. This model differs
from the textbook in two ways:

- **Rate independence.** The variances grow with the absolute (not squared)
  increments. The error then behaves like a random walk in distance and
  rotation, independent of the 50 Hz update rate.
- **No "rotation towards the motion".** The textbook's rot1 is undefined
  when the robot turns on the spot. With the simulator's 0.1 mm jitter it
  produced huge noise in the first version and made the pose jump. A
  regression test covers this case.

### 4.5 Sensor fusion and localisation (known initial pose)

1. **EKF** (`robot_localization`, 2D mode) fuses the noisy wheel odometry
   (vx, vyaw) and the IMU yaw rate. It publishes `odom → base_footprint`.
2. **AMCL** (likelihood-field laser model, 300–2000 particles) corrects the
   drift against the known map. It publishes `map → odom`.
   - It is initialised with the robot's **known home pose** (σ = 0.1 m).
   - Because the pieces are not in the map, `z_rand = 0.2` accounts for
     unexpected obstacles.
   - The arena has four different corner landmarks, so its square shape cannot
     cause an ambiguous localisation.

### 4.6 Map

- **Simulation:** `generate_arena.py` writes the Gazebo world and the occupancy
  grid (0.05 m/cell) from the same geometric description, so the map and the
  world match exactly. CI checks that the committed files are up to date.
- **Real robot:** the laboratory is mapped once with `gmapping`
  (`robot_chess_navigation/gmapping.launch`). The board position in that map
  is entered in `board_real.yaml`.

### 4.7 Visual piece verification

Each piece carries a saturated colour band at the camera height (0.12–0.24 m).
The band colour encodes the type; the hues are 60° apart. The body colour
(white or black) encodes the side. `robot_chess.vision.detect_piece()` runs
these steps:

1. take the central vertical strip of the image (the robot faces the piece);
2. threshold in HSV (S ≥ 110, V ≥ 60), then apply a morphological opening;
3. take the largest connected component as the band;
4. compute the circular mean hue and assign the nearest type hue
   (confidence = 1 − distance/15);
5. take the median brightness of the low-saturation pixels just below the band
   to decide white or black.

The `piece_vision` node classifies 5 consecutive frames and returns the
majority vote. A mismatch is reported in the event log. With
`abort_on_mismatch` set, the move stops instead.

### 4.8 Carrying pieces

The LIMO PRO has no manipulator. The `chess_carrier` interface hides how a
piece is moved:

- **Simulation:** the piece model is set onto the robot's carrying plate and
  follows the robot at 30 Hz until it is released on the target square.
- **Real robot:** `manual_carrier.py` publishes an instruction for a lab
  assistant ("put the WHITE PAWN on the robot") and waits for
  `rosservice call /chess_carrier/confirm`.

The piece mover is identical in both cases.

### 4.9 Two players, two robots

- The robots wait at home poses south and north of the board, outside the ring
  road, so they never block each other.
- Only the robot of the side to move drives.
- The referee rejects moves out of turn and moves while a robot is busy.
- Consoles can run on any machine on the same ROS master.

On the real robot only one side has a robot. The other side is configured as
*virtual*: its moves are typed into the console and the human moves the piece by
hand.

## 5. Simulation environment

| Element | Size / position |
|---|---|
| walled arena | 11 m × 11 m, 0.6 m walls |
| chessboard | 8 × 0.8 m = 6.4 m, centred at the origin |
| ring road | 0.5 m outside the board edge |
| graveyards | x = −4.7 m (white pieces) / +4.7 m (black pieces), 15 slots, 0.4 m apart |
| home poses | white (0, −4.5) facing north, black (0, 4.5) facing south |
| landmarks | box, cylinder, cylinder pair and rotated box in the corners; two wall stubs |
| pieces | 32 static models, spawned at runtime, 0.32–0.48 m tall |

## 6. Testing

- **Unit tests** (`robot_chess_game/test`, pytest):
  - geometry and layout validation of both shipped configurations;
  - road network: every edge clear of every possible piece position, every
    square and graveyard slot reachable from both homes, routes stay on
    lanes when starting between nodes;
  - move planning: captures, en passant, both castlings, promotion with
    capture, a full game;
  - vision: every type and colour on synthetic noisy images;
  - noise model: zero-noise exactness, accumulation, rate independence,
    backward motion, turning on the spot;
  - SDF validity.
- **Integration test** (`robot_chess_bringup/scripts/smoke_test.py`):
  - starts the full two-robot simulation headless and checks that both robots
    react to `cmd_vel`;
  - plays `1. e4 e5 2. Nf3` through the referee;
  - checks every piece's final pose in Gazebo (< 5 cm) and that the camera
    recognised every piece;
  - prints ground truth against estimated poses during the game.
- GitHub Actions runs both on every push, the second inside the
  `osrf/ros:noetic-desktop-full` container.

## 7. Real robot deployment

See README, section *Real LIMO PRO deployment*. In short:

1. drivers;
2. gmapping;
3. board pose in `board_real.yaml`;
4. `real_robot.launch color:=white`;
5. two consoles;
6. the assistant confirms load/unload.

The laboratory board uses 0.6 m squares with thinner pieces (10 cm diameter).
The layout is validated by the same unit tests.

## 8. Limitations and possible extensions

- A failed move is not rolled back physically. The referee reports it, and
  the player fixes the board and resubmits.
- Picking is virtual in simulation and manual on the real robot. A LIMO with
  the myCobot arm (`limo_cobot_sim`) could replace the carrier without changing
  the piece mover.
- The robots only drive on the road network. An unexpected obstacle on a lane
  makes the robot wait instead of planning around it.
- Chess960 castling is not supported (standard chess only).
- A chess engine could be added as a third "player" for single-human games.
