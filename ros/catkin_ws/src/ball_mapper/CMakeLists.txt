cmake_minimum_required(VERSION 2.8.3)
project(ball_mapper)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  ball_msgs
  message_generation
  message_runtime
  tf2_ros
  tf2_geometry_msgs
)

add_service_files(
  FILES
  ClosestBall.srv
)

generate_messages(
  DEPENDENCIES std_msgs geometry_msgs
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES ball_mapper
#  CATKIN_DEPENDS geometry_msgs roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(ball_tracker
  src/${PROJECT_NAME}/ball_tracker.cpp
)

add_dependencies(ball_tracker ${catkin_EXPORTED_TARGETS})
add_executable(${PROJECT_NAME}_node src/ball_mapper_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp ball_tracker)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ball_tracker
)
