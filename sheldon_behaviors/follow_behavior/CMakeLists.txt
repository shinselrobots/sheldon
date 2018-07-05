cmake_minimum_required(VERSION 2.8.3)
project(follow_behavior)

## Find catkin dependencies
find_package(catkin REQUIRED COMPONENTS pluginlib roscpp rospy behavior_common)

add_definitions(-std=c++11)

## Define catkin exports
catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS pluginlib roscpp behavior_common
  DEPENDS Boost
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/behavior_plugin.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY scripts DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES behavior_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
