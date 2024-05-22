cmake_minimum_required(VERSION 3.10)
project(global_planning)

set(ADDITIONAL_CXX_FLAG "-Wall -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ADDITIONAL_CXX_FLAG}")
# Eigen
include(cmake/FindEigen.cmake)
include_directories(${EIGEN_INCLUDE_DIR})

# glog
include(cmake/glog.cmake)
include_directories(${GLOG_INCLUDE_DIRS})
message(STATUS "GLOG_INCLUDE_DIRS: ${GLOG_INCLUDE_DIRS}")
message(STATUS "GLOG_LIBRARIES: ${GLOG_LIBRARIES}")

include_directories(
    include
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/cmake/geographiclib")
ament_auto_find_build_dependencies()
find_package(GeographicLib REQUIRED)

set(dependencies
  rclcpp
  geometry_msgs
  visualization_msgs
  std_msgs
  message_filters
  sensor_msgs
  nav_msgs
  tf2_ros
  GeographicLib
  planning_interfaces
  nav_msgs
  diagnostic_msgs
)


# Add library
add_library(
    PATH_SEARCHER_LIB SHARED
    src/global_planners/hybrid_a_star/rs_path.cpp
    src/global_planners/hybrid_a_star/costmap_subscriber.cpp
    src/global_planners/hybrid_a_star/goal_pose_subscriber.cpp
    src/global_planners/hybrid_a_star/init_pose_subscriber.cpp
    src/global_planners/hybrid_a_star/hybrid_a_star.cpp
)

# Link library with dependencies
target_link_libraries(
    PATH_SEARCHER_LIB
    ${GLOG_LIBRARIES}
)

ament_target_dependencies(PATH_SEARCHER_LIB
  ${dependencies}
)

ament_auto_add_executable(global_planner_manager 
  src/global_planner_manager.cpp 
  src/global_planners/race_planner.cpp
  src/global_planners/parking_planner.cpp
  src/global_planners/potential_field_algo.cpp

  src/global_planners/nav_planner.cpp
  src/global_planners/hybrid_a_star/hybrid_a_star_algo.cpp
  src/global_planners/hybrid_a_star/rs_path.cpp
  src/global_planners/hybrid_a_star/costmap_subscriber.cpp
  src/global_planners/hybrid_a_star/goal_pose_subscriber.cpp
  src/global_planners/hybrid_a_star/init_pose_subscriber.cpp
  src/global_planners/hybrid_a_star/hybrid_a_star.cpp
)

ament_target_dependencies(global_planner_manager ${dependencies})
target_link_libraries(global_planner_manager PATH_SEARCHER_LIB)

install(DIRECTORY include DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY param DESTINATION share/${PROJECT_NAME})

ament_auto_package()
