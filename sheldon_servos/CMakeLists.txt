cmake_minimum_required(VERSION 2.8.3)
project(sheldon_servos)

find_package(catkin REQUIRED COMPONENTS message_generation rospy)

# generate_messages()
catkin_package(
  CATKIN_DEPENDS dynamixel_controllers 
)

install(DIRECTORY launch config udev scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


catkin_install_python(PROGRAMS scripts/record_servos.py scripts/shared_scripts/head_servo_publishers.py  scripts/shared_scripts/dynamixel_joint_state_publisher.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


catkin_python_setup()





