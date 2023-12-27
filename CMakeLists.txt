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
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
  FILES_MATCHING PATTERN "*.world"
  PATTERN ".svn" EXCLUDE
)

catkin_install_python(PROGRAMS src/ros/moveit_tutorial.py src/ros/lire_json.py src/ros/position_magnet.py src/ros/marker.py src/ros/marker_subscriber.py src/ros_action/pos_magnet_client.py src/ros_action/marker_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  config/bounding.json
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

