cmake_minimum_required(VERSION 3.0.2)
project(rosi_seed_noid_sim)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  gazebo_ros
  rois_env
  rospy
  seed_r7_gazebo
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS cv_bridge gazebo_ros rois_env rospy seed_r7_gazebo sensor_msgs std_msgs
)

catkin_install_python(
  PROGRAMS scripts/person_detection_hog.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_repository_layout.py)
endif()
