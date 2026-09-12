cmake_minimum_required(VERSION 3.8)
project(opponent_tracker)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}_core
  src/cluster_extractor.cpp
  src/kalman_tracker_2d.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
)
target_link_libraries(${PROJECT_NAME}_core
  Eigen3::Eigen
)

add_executable(opponent_tracker_node
  src/opponent_tracker_node.cpp
  src/main.cpp
)
target_link_libraries(opponent_tracker_node
  ${PROJECT_NAME}_core
  Eigen3::Eigen
)
ament_target_dependencies(opponent_tracker_node
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
)

install(TARGETS
  ${PROJECT_NAME}_core
  opponent_tracker_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

