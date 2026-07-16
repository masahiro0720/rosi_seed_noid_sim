cmake_minimum_required(VERSION 3.0.2)
project(rosi_seed_noid_sim)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  seed_r7_gazebo
)

catkin_package(
  CATKIN_DEPENDS gazebo_ros seed_r7_gazebo
)

install(
  DIRECTORY launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_repository_layout.py)
endif()
