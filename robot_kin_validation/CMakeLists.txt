cmake_minimum_required(VERSION 3.8)
project(robot_kin_validation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

## INCLUDE(S)
include_directories(include)

# executables
add_executable(robot_kin_validation src/robot_kin_validation.cpp)

# dependencies
ament_target_dependencies(robot_kin_validation rclcpp moveit_ros_planning_interface tf2 tf2_ros)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS robot_kin_validation
  DESTINATION lib/${PROJECT_NAME})

ament_package()
