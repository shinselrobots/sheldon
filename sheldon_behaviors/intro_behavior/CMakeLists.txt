cmake_minimum_required(VERSION 2.8.3)
project(intro_behavior)

## Find catkin dependencies
find_package(catkin REQUIRED COMPONENTS pluginlib roscpp behavior_common audio_and_speech_common)

## Find Boost (headers only)
find_package(Boost REQUIRED)

## Define catkin exports
catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS pluginlib roscpp behavior_common audio_and_speech_common
  DEPENDS Boost
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_definitions(-std=c++11)

add_library(${PROJECT_NAME} src/behavior_plugin.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES behavior_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
