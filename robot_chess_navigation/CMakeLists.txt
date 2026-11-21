cmake_minimum_required(VERSION 3.0.2)
project(robot_chess_navigation)

find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(PROGRAMS scripts/noisy_odometry.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
