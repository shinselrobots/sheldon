To run tests:

roslaunch sheldon robot.launch
  if not in robot.launch: roslaunch sheldon_servos servos.launch

roslaunch sheldon_moveit_config move_group.launch


run the test:

roslaunch sheldon_description view_robot.launch

rosrun moveit_commander moveit_commander_cmdline.py 



