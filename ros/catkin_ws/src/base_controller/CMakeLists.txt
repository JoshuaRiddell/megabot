cmake_minimum_required(VERSION 2.8.3)
project(base_controller)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  actionlib_msgs
  actionlib
  tf2_ros
)

add_action_files(
  DIRECTORY action
  FILES GotoPoint.action ResetOdom.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs geometry_msgs sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(speed_curve src/base_controller/speed_curve.cpp)
add_library(acceleration_limiter src/base_controller/acceleration_limiter.cpp)
add_library(goto_action src/base_controller/goto_action.cpp)
add_executable(${PROJECT_NAME}_node src/base_controller_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  speed_curve
  acceleration_limiter
  goto_action
)
target_link_libraries(speed_curve ${catkin_LIBRARIES})
target_link_libraries(acceleration_limiter ${catkin_LIBRARIES})
target_link_libraries(goto_action ${catkin_LIBRARIES})


