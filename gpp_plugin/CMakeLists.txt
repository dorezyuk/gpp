cmake_minimum_required(VERSION 3.0.2)
project(gpp_plugin)

# define the required components
set(catkin_PACKAGES costmap_2d gpp_interface mbf_costmap_core nav_core pluginlib xmlrpcpp)

find_package(catkin REQUIRED COMPONENTS ${catkin_PACKAGES})

catkin_package(
 INCLUDE_DIRS src
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS ${catkin_PACKAGES}
)

add_library(${PROJECT_NAME}  src/${PROJECT_NAME}.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC ${catkin_INCLUDE_DIRS} src)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY src/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  # gtests
  catkin_add_gtest(${PROJECT_NAME}_test test/load_plugin.cpp)
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})

  # ros-tests
  add_rostest_gmock(array_plugin_manager_test
    test/array_plugin_manager.launch
    test/array_plugin_manager.cpp)
  target_link_libraries(array_plugin_manager_test ${PROJECT_NAME})

  add_rostest(test/mbf_costmap_nav.launch)
  add_rostest(test/move_base.launch)

endif()
