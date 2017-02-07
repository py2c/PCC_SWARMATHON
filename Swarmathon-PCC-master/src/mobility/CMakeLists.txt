cmake_minimum_required(VERSION 1.8.3)
project(mobility)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  message_generation
  random_numbers
  rospy
)

# We do not have services (functions that can be called externally)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs random_numbers message_runtime
)

catkin_python_setup()

catkin_install_python(PROGRAMS bin/mobility
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

