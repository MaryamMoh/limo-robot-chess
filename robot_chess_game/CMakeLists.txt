cmake_minimum_required(VERSION 3.0.2)
project(robot_chess_game)

find_package(catkin REQUIRED COMPONENTS robot_chess_msgs)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS robot_chess_msgs)

file(GLOB NODE_SCRIPTS ${PROJECT_SOURCE_DIR}/scripts/*.py)
catkin_install_python(PROGRAMS ${NODE_SCRIPTS}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_program(PYTEST NAMES pytest-3 pytest)
  if(PYTEST)
    add_test(NAME robot_chess_game_pytest
      COMMAND ${PYTEST} -q ${PROJECT_SOURCE_DIR}/test
      WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
    set_tests_properties(robot_chess_game_pytest PROPERTIES
      ENVIRONMENT "PYTHONPATH=${PROJECT_SOURCE_DIR}/src")
  endif()
endif()
