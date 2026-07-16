cmake_minimum_required(VERSION 3.0.2)
project(rosi_seed_noid_sim)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  cv_bridge
  gazebo_ros
  global_planner
  geometry_msgs
  message_generation
  move_base
  move_base_msgs
  rois_env
  rospy
  seed_r7_gazebo
  seed_r7_navigation
  sensor_msgs
  std_msgs
  teb_local_planner
  tf2_geometry_msgs
  tf2_ros
)

add_message_files(
  FILES
  PersonRoi.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib actionlib_msgs cv_bridge gazebo_ros global_planner
  geometry_msgs message_runtime move_base move_base_msgs rois_env rospy
  seed_r7_gazebo seed_r7_navigation sensor_msgs std_msgs teb_local_planner
  tf2_geometry_msgs tf2_ros
)

catkin_install_python(
  PROGRAMS
  scripts/move_hri_sim.py
  scripts/navigation_hri_sim.py
  scripts/person_detection_hog.py
  scripts/person_localization_depth.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY config launch models worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_depth_geometry.py)
  catkin_add_nosetests(test/test_repository_layout.py)
endif()
