cmake_minimum_required(VERSION 2.8.3)
project(sheldon)

find_package(catkin REQUIRED COMPONENTS message_generation rospy)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch config param
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
   scripts/activated_tensorflow.sh
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

