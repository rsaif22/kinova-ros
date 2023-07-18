cmake_minimum_required(VERSION 3.0.2)
project(kinova_gazebo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(fast_joints_plugin)

catkin_package(
  CATKIN_DEPENDS roscpp gazebo_ros fast_joints_plugin
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

