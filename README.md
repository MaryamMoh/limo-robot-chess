# LIMO Robot Chess

[![CI](https://github.com/MaryamMoh/limo-robot-chess/actions/workflows/ci.yml/badge.svg)](https://github.com/MaryamMoh/limo-robot-chess/actions/workflows/ci.yml)

A chess-playing robot: an AgileX **LIMO** with a **myCobot 280** arm, playing
against another robot on a real chessboard.

Two robots sit on opposite sides of the board. Each one runs its own
software; this repository is **my** robot. They share nothing but the board:

- My robot **sees** the board with a wrist camera.
- It keeps the **state of the game**: position, legal moves, check, mate, draws.
- It **recognises the opponent's move** when the other arm has finished.
- It **chooses the best reply** with its own alpha-beta engine, or Stockfish.
- It **plays it with the arm**, putting captured pieces beside the board.

Everything is tested in Gazebo first. In the simulation a stand-in plays the
other side the way the opponent robot would. The same nodes then run on the
real robot in the laboratory.

![architecture](docs/architecture.svg)

## How a game runs

1. **Start.** The arm moves the camera above the board. The robot learns the
   board's colours from the start position, so it adapts to the real board and
   lighting.
2. **My turn.**
   - The engine picks the best move.
   - The move is split into arm tasks: capture to the graveyard first, then the
     moving piece, plus the rook when castling and the piece exchange when
     promoting.
   - The arm executes the tasks with straight, top-down tool paths.
   - The camera checks the board matches the expected position.
3. **Opponent's turn.**
   - The camera reads the board every second.
   - Once the board has changed and stayed the same for 3 readings (the other
     arm is done), the robot finds the **legal move that explains the change**.
     That covers captures, castling, en passant and promotion.
   - An unexplained change, such as a half-finished move or an illegal one, is
     reported and waited out.
4. **Game end.** Checkmate, stalemate, draws by rule and threefold repetition
   end the game.

## Packages

| Package | Contents |
|---|---|
| `robot_chess_game` | the player: `chess_player` (brain), `arm_executor`, `board_vision_node`, `mycobot_driver` (real arm), `board_visualizer`, `chess_monitor`, `check_reach`; ROS-independent library `src/robot_chess` (kinematics, engine, vision, move inference) with unit tests |
| `robot_chess_description` | LIMO + myCobot 280 model with gripper and wrist camera, controllers |
| `robot_chess_gazebo` | generated table/board world, pieces with virtual grasp, opponent stand-in |
| `robot_chess_bringup` | `simulation.launch`, `real_robot.launch`, RViz view, end-to-end test |
| `robot_chess_msgs` | game state, board observation, arm action, services |

## Installation (Ubuntu 20.04, ROS Noetic)

```bash
sudo apt install ros-noetic-desktop-full ros-noetic-ros-controllers \
     ros-noetic-gazebo-ros-control ros-noetic-usb-cam python3-pip
pip3 install chess
sudo apt install stockfish            # optional, stronger engine

mkdir -p ~/chess_ws/src && cd ~/chess_ws/src
git clone https://github.com/MaryamMoh/limo-robot-chess.git
git clone https://github.com/agilexrobotics/limo_ros.git     # LIMO meshes and base driver
cd ~/chess_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make && source devel/setup.bash
```

## Testing in simulation

```bash
roslaunch robot_chess_bringup simulation.launch           # Gazebo + RViz + both sides
rosrun robot_chess_game chess_monitor.py                  # board, status, engine line
```

```
    a  b  c  d  e  f  g  h
8 | r  n  b  q  k  b  n  r |
...
moves : 1. e4 e5 2. Nf3
status: waiting for the opponent
engine: internal  +35 cp  depth 4  line Nf3 Nc6 Bb5 a6
```

Options:
- `opponent_strategy:=random` gives a weaker, unpredictable opponent.
- `opponent:=false` lets you play the other side yourself, by moving the
  pieces in Gazebo.
- `gui:=false` runs headless.
- `rosservice call /chess_player/override_opponent_move "move: 'e8=N'"` tells
  the player the opponent's move when the camera cannot tell, for example an
  under-promotion.

RViz shows the robot, the board as the player believes it to be, the engine's
line, and the camera image with the classification of each square.

## Physical setup and reach

The myCobot 280 reaches only about 7–27 cm from its base with the gripper
pointing down, and the LIMO body sticks out 16 cm in front of the arm. The
simulated setup therefore uses:

- **Board:** a travel-size board (2 cm squares, 16 cm) on a low table whose top
  (22 cm) overhangs the robot's front bumper. The nearest rank is 10 cm from
  the arm axis.
- **Graveyard:** 3 × 5 slots beside the h-file, for captured pieces.
- **Wrist camera:** wide angle (86°), mounted 4 cm ahead of the tool axis and
  tilted away from the fingers. It reads the board from an observe pose 13 cm
  above it.

Check any layout before a lab session:

```bash
rosrun robot_chess_game check_reach.py robot_chess_game/config/player_real.yaml
```

```
reachable squares (o) / unreachable (X), rank 8 at the top:
  8 o o o o o o o o
  ...
=> layout OK
```

**Bigger boards:** the LIMO can drive straight forward and back under the table
to *base stations*, for example `base: {stations: [0.0, 0.06, 0.12]}` for a
3 cm board. The arm executor picks a station from which each square is
reachable, and the camera always reads the board from the home station.

## Real robot (laboratory)

1. Install `pymycobot` (`pip3 install pymycobot`) and connect the camera to
   the wrist mount described in the URDF (`wrist_camera_joint`), or update the
   URDF to where it really is.
2. Calibrate the camera once (start `real_robot.launch`, then):
   ```bash
   rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.024 \
          image:=/wrist_camera/image_raw camera:=/wrist_camera
   ```
3. Measure where the board is relative to the robot. Fill in
   `robot_chess_game/config/player_real.yaml`: board pose, colour, tool length
   `tcp_offset`. Then run `check_reach.py` on it.
4. Set up the start position and launch:
   ```bash
   roslaunch robot_chess_bringup real_robot.launch rviz:=true
   rosrun robot_chess_game chess_monitor.py
   ```
   Don't touch the board until the robot reports "ready": it is learning the
   board colours.
5. If the robot promotes a pawn, it asks you (`/chess/operator`) to exchange the
   piece. Confirm with `rosservice call /arm_executor/confirm`.

`mycobot_driver.py` offers the same trajectory interfaces as the simulated
controllers, so the arm code is identical in both worlds.

## Tests

```bash
cd robot_chess_game && PYTHONPATH=src python3 -m pytest -q test
roslaunch robot_chess_bringup simulation.launch gui:=false rviz:=false &
rosrun robot_chess_bringup smoke_test.py _plies:=6
```

The unit tests cover:
- the kinematics against the URDF zero pose, IK round trips and layout reach;
- engine tactics: mates, winning material, not blundering the queen;
- recognising every kind of move;
- board reading on rendered camera images;
- move planning and base stations.

GitHub Actions runs these and also plays six half-moves in Gazebo. It checks
that both robots agree on the game and that the pieces in Gazebo match the
player's position.

## Limitations

- The camera sees occupancy (empty, white, black), not piece types. The game
  history fills that in. An opponent's under-promotion therefore needs the
  override service, since a queen is assumed.
- In Gazebo the gripper holds pieces kinematically ("virtual grasp"), because
  simulated grasping of 1.3 cm pieces is unreliable. The arm still has to
  bring the fingers exactly to the piece.
- On the real robot, the camera mount and tool length must match the URDF and
  config. The real arm path (`mycobot_driver.py`) has not been tested on
  hardware yet.

See [docs/system_design.md](docs/system_design.md) for the full design.
