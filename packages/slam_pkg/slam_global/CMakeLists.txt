cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0074 NEW)
project(slam_global)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------------------------------
# Dependencies
# ---------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pcl_conversions REQUIRED)

# PCL itself
find_package(PCL REQUIRED COMPONENTS common io filters registration)

# ---------------------------------
# Library with all core modules
# ---------------------------------
add_library(slam_global_lib
  src/MapLoader.cpp
  src/SensorManager.cpp
  src/LocalizerICP.cpp  
  src/SubmapExtractor.cpp
  src/MapToOdomPublisher.cpp
)

ament_target_dependencies(slam_global_lib
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  Eigen3
  pcl_conversions
)

target_include_directories(slam_global_lib
  PRIVATE
    include
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(slam_global_lib
  ${PCL_LIBRARIES}
)

target_compile_definitions(slam_global_lib
  PRIVATE ${PCL_DEFINITIONS}
)

# ---------------------------------
# Node executable (the main)
# ---------------------------------
add_executable(localizer_node src/localizer_node.cpp)
add_executable(transform_node src/transform_node.cpp)
add_executable(odom2pose src/odom2pose.cpp)

ament_target_dependencies(localizer_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  Eigen3
  pcl_conversions
)

ament_target_dependencies(transform_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
  Eigen3
)

ament_target_dependencies(odom2pose
  rclcpp
  nav_msgs
  geometry_msgs
)

target_include_directories(localizer_node
  PRIVATE
    include
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

target_include_directories(transform_node
  PRIVATE
    include
    ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(localizer_node
  slam_global_lib
  ${PCL_LIBRARIES}
)

target_compile_definitions(localizer_node
  PRIVATE ${PCL_DEFINITIONS}
)

# ---------------------------------
# Install targets
# ---------------------------------
install(TARGETS
  localizer_node
  slam_global_lib
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  transform_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  odom2pose
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/pub_initpose.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
