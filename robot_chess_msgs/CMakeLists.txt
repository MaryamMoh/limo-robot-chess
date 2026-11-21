cmake_minimum_required(VERSION 3.0.2)
project(robot_chess_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  actionlib_msgs
  std_msgs
  geometry_msgs
)

add_message_files(FILES
  PieceTask.msg
  GameState.msg
)

add_service_files(FILES
  SubmitMove.srv
  VerifyPiece.srv
  AttachPiece.srv
  ReleasePiece.srv
  ReplacePiece.srv
)

add_action_files(FILES
  ExecuteMove.action
)

generate_messages(DEPENDENCIES
  actionlib_msgs
  std_msgs
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime actionlib_msgs std_msgs geometry_msgs)
