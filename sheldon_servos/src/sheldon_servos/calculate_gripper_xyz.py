#!/usr/bin/env python
# Calculate gripper position given shoulder pose and arm servo angles
# can be used for real position (but usually get directly from "get_gripper_xyz")
# but mostly used for arm movement planning

import sys
import roslib
import rospy, time

import tf
from tf import TransformListener
#import moveit_commander
#from moveit_commander import MoveGroupCommander
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import geometry_msgs.msg
# use numpy?

class GetGripperPose():
    def __init__(self):
                       
        rospy.init_node('GetGripperPose')

    # returns absolute gripper_link xyz position from base_link
    def CalculateXYZ( right_arm,
        shoulder_position_x, 
        shoulder_position_y, 
        shoulder_position_z, 
        right_arm_shoulder_rotate_angle,
        right_arm_shoulder_lift_angle,
        right_arm_elbow_rotate_angle,
        right_arm_elbow_bend_angle,
        right_arm_wrist_bend_angle):
    
        gripper_x = 0.0
        gripper_y = 0.0
        gripper_z = 0.0


 


if __name__=='__main__':

    try:
        GetGripperPose()
    except rospy.ROSInterruptException:
        rospy.loginfo("Oops! Exception occurred.") 






