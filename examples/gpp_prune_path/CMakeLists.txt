cmake_minimum_required(VERSION 3.0.2)
project(gpp_prune_path)

set(catkin_PACKAGES gpp_interface pluginlib)
find_package(catkin REQUIRED COMPONENTS ${catkin_PACKAGES})

catkin_package(
 INCLUDE_DIRS src
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS ${catkin_PACKAGES}
)

add_library(${PROJECT_NAME} src/gpp_prune_path.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC src ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

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
  catkin_add_gtest(${PROJECT_NAME}_test test/prune.cpp)
  target_include_directories(${PROJECT_NAME}_test PRIVATE src)
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
endif()
