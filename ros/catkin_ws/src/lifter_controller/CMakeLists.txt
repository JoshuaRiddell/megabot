cmake_minimum_required(VERSION 2.8.3)
project(lifter_controller)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/Lifter.cfg
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES coordinate_transformer
#  CATKIN_DEPENDS geometry_msgs roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/lifter_controller_node.cpp)

add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
