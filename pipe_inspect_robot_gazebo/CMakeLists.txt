cmake_minimum_required(VERSION 3.0.2)
project(pipe_inspect_robot_gazebo)

find_package(catkin REQUIRED)

find_package(PCL REQUIRED) # This includes all modules
find_package(PkgConfig)
pkg_check_modules(EIGEN3 eigen3 REQUIRED)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  joy
  pcl_conversions
  pcl_ros
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  pipe_direction_options.msg
#  Message1.msg
#   Message2.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pipe_inspect_robot_gazebo
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(simulated_sl_sensor src/simulated_sl_sensor.cpp src/sensor_data_structure.cpp)
target_link_libraries(simulated_sl_sensor ${catkin_LIBRARIES})

