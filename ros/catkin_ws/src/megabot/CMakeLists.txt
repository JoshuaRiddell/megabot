cmake_minimum_required(VERSION 2.8.3)
project(megabot)

find_package(catkin REQUIRED COMPONENTS
  xacro
  hector_sensors_description
)

catkin_package()

install(
  DIRECTORY urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
