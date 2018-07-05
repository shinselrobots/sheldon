#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Float64
import random

# 90 degrees = 1.57, 45 = 0.785

# global


def left_claw_open_half():
    print("-----> left_claw_open_half")
    pub_left_arm_claw.publish(0.8)

def left_claw_open():
    print("-----> left_claw_open")
    pub_left_arm_claw.publish(0.3)

def left_claw_close():
    print("-----> left_claw_close")
    pub_left_arm_claw.publish(1.5)


# ========================================================================================
# Main

if __name__ == '__main__':
    pub_pan = rospy.Publisher('/head_pan_controller/command', Float64, queue_size=1)
    pub_tilt = rospy.Publisher('/head_tilt_controller/command', Float64, queue_size=1)
    pub_sidetilt = rospy.Publisher('/head_sidetilt_controller/command', Float64, queue_size=1)

    pub_left_arm_lift = rospy.Publisher('/left_arm_lift_controller/command', Float64, queue_size=1)
    pub_left_arm_elbow_rotate = rospy.Publisher('/left_arm_elbow_rotate_controller/command', Float64, queue_size=1)
    pub_left_arm_elbow_bend = rospy.Publisher('/left_arm_elbow_bend_controller/command', Float64, queue_size=1)
    pub_left_arm_wrist_rotate = rospy.Publisher('/left_arm_wrist_rotate_controller/command', Float64, queue_size=1)
    pub_left_arm_claw = rospy.Publisher('/left_arm_claw_controller/command', Float64, queue_size=1)
    pub_left_arm_shoulder = rospy.Publisher('/left_arm_shoulder_controller/command', Float64, queue_size=1)

    pub_right_arm_lift = rospy.Publisher('/right_arm_lift_controller/command', Float64, queue_size=1)
    pub_right_arm_elbow_rotate = rospy.Publisher('/right_arm_elbow_rotate_controller/command', Float64, queue_size=1)
    pub_right_arm_elbow_bend = rospy.Publisher('/right_arm_elbow_bend_controller/command', Float64, queue_size=1)
    pub_right_arm_wrist_rotate = rospy.Publisher('/right_arm_wrist_rotate_controller/command', Float64, queue_size=1)
    pub_right_arm_claw = rospy.Publisher('/right_arm_claw_controller/command', Float64, queue_size=1)
    pub_right_arm_shoulder = rospy.Publisher('/right_arm_shoulder_controller/command', Float64, queue_size=1)



# Initialize node for ROS
rospy.init_node('listener', anonymous=True) # TODO change this!

print();
print("claw open...")


##### START #####

left_claw_open()




