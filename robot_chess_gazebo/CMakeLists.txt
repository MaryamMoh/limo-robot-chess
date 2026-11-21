cmake_minimum_required(VERSION 3.0.2)
project(robot_chess_gazebo)

find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(PROGRAMS
  scripts/sim_piece_manager.py
  scripts/opponent_simulator.py
  scripts/generate_world.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
