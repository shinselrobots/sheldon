# sheldon_description
## Model of the robot (URDF and meshes) and tests (launch and rviz) 

##Test URDF model:
  For verifying URDF is correct
    - sets rviz to use "base_link"
    - launchs "joint_state_publisher" to simulate robot state that would normally come from the real robot

    roslaunch sheldon_description test_model.launch


## View live model of running robot:
  To view robot pose, feedback from servos, sensors, etc.
    - this is the state that the robot thinks is is in.

    roslaunch sheldon robot.launch
    roslaunch sheldon_description view_model.launch


## IMPORTANT:
the URDF is used for MOVE_IT!
    - "robot.urdf.xacro" is used for everything except for MoveIt
    - "sheldon.urdf" is created from "robot.urdf.xacro", and is used for MoveIt

If you modify the URDF re-run the following:
    cd urdf
    rosrun xacro xacro --inorder robot.urdf.xacro > sheldon.urdf
    roslaunch moveit_setup_assistant setup_assistant.launch and edit as needed


## Optional:
    sudo apt install liburdfdom-tools
    check_urdf sheldon.urdf
