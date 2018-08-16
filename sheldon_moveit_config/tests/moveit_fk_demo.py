#!/usr/bin/env python

"""
    moveit_fk_demo.py - Version 0.1 2014-01-14
    
    Use forward kinemtatics to move the arm to a specified set of joint angles
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2014 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import rospy, sys
import moveit_commander
from control_msgs.msg import GripperCommand

class MoveItDemo:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the ROS node
        rospy.init_node('moveit_demo', anonymous=True)

        # Connect to the right_arm move group
        right_arm = moveit_commander.MoveGroupCommander('right_arm')
        
        # Set a small tolerance on joint angles
        right_arm.set_goal_joint_tolerance(0.001)
        
        # Start the arm target in "resting" pose stored in the SRDF file
        right_arm.set_named_target('right_arm_home')
        
        # Plan a trajectory to the goal configuration
        traj = right_arm.plan()
         
        # Execute the planned trajectory
        right_arm.execute(traj)
        
        # Pause for a moment
        rospy.sleep(2)
         
                 
        # Set the arm target to the named "straight_out" pose stored in the SRDF file
        right_arm.set_named_target('right_arm_extend_full')
         
        # Plan and execute the motion
        right_arm.go()
        rospy.sleep(5)
                  
        
        # Return the arm to the named "resting" pose stored in the SRDF file
        right_arm.set_named_target('right_arm_home')
        right_arm.go()
        rospy.sleep(2)
         
       
        # Cleanly shut down MoveIt
        moveit_commander.roscpp_shutdown()
        
        # Exit the script
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItDemo()
    except rospy.ROSInterruptException:
        pass
