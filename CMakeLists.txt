cmake_minimum_required(VERSION 3.0.2)
project(hwt9053_ros_driver)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  serial
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hwt9053_ros_driver
#  CATKIN_DEPENDS roscpp rospy sensor_msgs serial std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(hwt9053_driver_node
  src/hwt9053_driver_node.cpp
  src/modbus_rtu_master.cpp
)
add_dependencies(hwt9053_driver_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(hwt9053_driver_node
  ${catkin_LIBRARIES}
)