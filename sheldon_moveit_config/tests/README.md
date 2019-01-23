# To run tests:

- roslaunch sheldon robot.launch
  if not in robot.launch: roslaunch sheldon_servos servos.launch

- roslaunch sheldon_moveit_config move_group.launch


run manual tests:
(see Ros By Example, section 11.14 Testing Moveit Command Line)

- roslaunch sheldon_description view_model.launch

- rosrun moveit_commander moveit_commander_cmdline.py 
  > use right_arm
  > current
    (displays current position of arm)
  > go right_arm_home
  > go right_arm_extend
  > go random  (make sure robot work area is clear!)
  > exit

  - some pre-defined target positions (right or left arm)
    (see sheldon_robot.srdf, "group_state")
    right_arm_relaxed
    right_arm_extend
    right_arm_extend_full
    right_arm_home


# Run python tests: 
cd tests
./moveit_fk_demo.py
./moveit_ik_demo.py


