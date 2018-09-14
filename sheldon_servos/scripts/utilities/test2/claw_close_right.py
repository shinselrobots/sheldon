#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Float64
import random

# 90 degrees = 1.57, 45 = 0.785

# global

# ================================================================


# RIGHT

def right_gripper_open_half():
    print("-----> right_gripper_open")
    pub_right_arm_gripper_finger.publish(-1.0)

def right_gripper_close():
    print("-----> right_gripper_close")
    pub_right_arm_gripper_finger.publish(-2.0)




# ========================================================================================
# Main

if __name__ == '__main__':
    pub_pan = rospy.Publisher('/head_pan_joint/command', Float64, queue_size=1)
    pub_tilt = rospy.Publisher('/head_tilt_joint/command', Float64, queue_size=1)
    pub_sidetilt = rospy.Publisher('/head_sidetilt_joint/command', Float64, queue_size=1)

    pub_left_arm_lift = rospy.Publisher('/left_arm_lift_joint/command', Float64, queue_size=1)
    pub_left_arm_elbow_rotate = rospy.Publisher('/left_arm_elbow_rotate_joint/command', Float64, queue_size=1)
    pub_left_arm_elbow_bend = rospy.Publisher('/left_arm_elbow_bend_joint/command', Float64, queue_size=1)
    pub_left_arm_wrist_rotate = rospy.Publisher('/left_arm_wrist_rotate_joint/command', Float64, queue_size=1)
    pub_left_arm_gripper_finger = rospy.Publisher('/left_arm_gripper_finger_joint/command', Float64, queue_size=1)
    pub_left_arm_shoulder = rospy.Publisher('/left_arm_shoulder_controller/command', Float64, queue_size=1)

    pub_right_arm_lift = rospy.Publisher('/right_arm_lift_joint/command', Float64, queue_size=1)
    pub_right_arm_elbow_rotate = rospy.Publisher('/right_arm_elbow_rotate_joint/command', Float64, queue_size=1)
    pub_right_arm_elbow_bend = rospy.Publisher('/right_arm_elbow_bend_joint/command', Float64, queue_size=1)
    pub_right_arm_wrist_rotate = rospy.Publisher('/right_arm_wrist_rotate_joint/command', Float64, queue_size=1)
    pub_right_arm_gripper_finger = rospy.Publisher('/right_arm_gripper_finger_joint/command', Float64, queue_size=1)
    pub_right_arm_shoulder = rospy.Publisher('/right_arm_shoulder_controller/command', Float64, queue_size=1)



# Initialize node for ROS
rospy.init_node('listener', anonymous=True) # TODO change this!

print("gripper...")


##### START #####
#right_gripper_open_half()
right_gripper_close()




