cmake_minimum_required(VERSION 3.0.2)
project(rosi_seed_noid_sim)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  gazebo_ros
  geometry_msgs
  message_generation
  rois_env
  rospy
  seed_r7_gazebo
  sensor_msgs
  std_msgs
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
  CATKIN_DEPENDS cv_bridge gazebo_ros geometry_msgs message_runtime rois_env rospy
  seed_r7_gazebo sensor_msgs std_msgs tf2_geometry_msgs tf2_ros
)

catkin_install_python(
  PROGRAMS
  scripts/person_detection_hog.py
  scripts/person_localization_depth.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_depth_geometry.py)
  catkin_add_nosetests(test/test_repository_layout.py)
endif()
