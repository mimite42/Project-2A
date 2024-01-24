cmake_minimum_required(VERSION 3.0.2)
project(test)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  roscpp
  rospy
  std_msgs
  control_msgs
  actionlib_msgs
)

# Ajoutez pose.action dans la liste des fichiers d'action à générer
add_action_files(
  FILES
  pose.action
  Move.action
  Robot.action
  ur_robot.action
)

# Déclarez que le package dépend de la génération des messages d'action
generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package()

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY
  launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
  PATTERN ".svn" EXCLUDE
)

install(FILES
  sigma_cell.xacro
  sigma.xacro
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
  FILES_MATCHING PATTERN "*.world"
  PATTERN ".svn" EXCLUDE
)

catkin_install_python(PROGRAMS script/moveit_tutorial.py script/myfile.py script/marker.py script/marker_publisher.py script/r_publisher.py script/test_publish_markerarray.py action_script/pr_move.py action_script/test_move.py action_script/pr_robot.py action_script/test_robot.py action_script/sk_UR_robot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

