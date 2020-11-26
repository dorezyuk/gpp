cmake_minimum_required(VERSION 3.0.2)
project(gpp_interface)

set(catkin_PACKAGES costmap_2d geometry_msgs)

find_package(catkin REQUIRED COMPONENTS ${catkin_PACKAGES})
catkin_package(INCLUDE_DIRS src CATKIN_DEPENDS ${catkin_PACKAGES})

install(DIRECTORY src/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
