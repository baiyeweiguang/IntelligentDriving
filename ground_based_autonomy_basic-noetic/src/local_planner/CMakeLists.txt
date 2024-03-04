cmake_minimum_required(VERSION 3.10)
project(local_planner)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Find catkin macros and libraries
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
# message(WWW)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(laser_geometry REQUIRED)

set(dependencies
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  nav_msgs
  pcl_ros
  laser_geometry
)


find_package(PCL REQUIRED)



###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${PCL_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)


## Declare executables
add_executable(localPlanner src/localPlanner.cpp)
add_executable(pathFollower src/pathFollower.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(localPlanner ${PCL_LIBRARIES})
target_link_libraries(pathFollower ${PCL_LIBRARIES})
ament_target_dependencies(localPlanner ${dependencies})
ament_target_dependencies(pathFollower ${dependencies})

install(DIRECTORY launch config paths
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS localPlanner pathFollower
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_export_dependencies(sensor_msgs)
ament_package()