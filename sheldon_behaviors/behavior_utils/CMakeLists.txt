cmake_minimum_required(VERSION 2.8.3)
project(behavior_utils)

## Find catkin dependencies
find_package(catkin REQUIRED COMPONENTS pluginlib roscpp actionlib actionlib_msgs message_generation)

## Find Boost (headers only)
find_package(Boost REQUIRED)

find_package(PythonLibs REQUIRED)

#add_message_files(
#  FILES
#  CommandState.msg
#)

## Generate actions in the 'action' folder
#add_action_files(
#  FILES
#  behavior.action
#)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs std_msgs
)

## Define catkin exports
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp actionlib actionlib_msgs message_generation
  DEPENDS Boost
  LIBRARIES behavior_utils
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS})

add_definitions(-std=c++11)

add_library(${PROJECT_NAME} SHARED src/behavior_utils.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PYTHON_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#add_executable(behavior_loader src/behavior_plugin_loader.cpp)
#target_link_libraries(behavior_loader ${catkin_LIBRARIES})
#add_dependencies(behavior_loader ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#install(TARGETS ${PROJECT_NAME} behavior_loader 
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

#install(DIRECTORY launch 
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

## Mark other files for installation (e.g. launch and bag files, etc.)
#install(FILES behavior_plugins.xml
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
