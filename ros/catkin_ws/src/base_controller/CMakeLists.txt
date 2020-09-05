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
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/BaseController.cfg
)

add_action_files(
  DIRECTORY action
  FILES GotoPoint.action GotoPose.action ResetOdom.action StepperPower.action
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
add_library(reset_odom_action src/base_controller/reset_odom_action.cpp)
add_library(stepper_power_action src/base_controller/stepper_power_action.cpp)
add_executable(${PROJECT_NAME}_node src/base_controller_node.cpp)
add_dependencies(goto_action ${PROJECT_NAME}_gencfg)
add_dependencies(reset_odom_action ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(stepper_power_action ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  speed_curve
  acceleration_limiter
  goto_action
  reset_odom_action
  stepper_power_action
)
target_link_libraries(speed_curve ${catkin_LIBRARIES})
target_link_libraries(acceleration_limiter ${catkin_LIBRARIES})
target_link_libraries(goto_action ${catkin_LIBRARIES} speed_curve acceleration_limiter)
target_link_libraries(reset_odom_action ${catkin_LIBRARIES})
target_link_libraries(stepper_power_action ${catkin_LIBRARIES})


