#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Float64
import random

# 90 degrees = 1.57, 45 = 0.785

# global

# ================================================================
# HEAD

def head_center():
    print("-----> head_center")
    pub_sidetilt.publish(0.0)
    pub_tilt.publish(0.0)
    pub_pan.publish(0.0)

def head_down():
    print("-----> head_down")
    pub_sidetilt.publish(0.0)
    pub_tilt.publish(1.26)
    pub_pan.publish(0.0)

def all_home(): # all in home position
    print("-----> all_home")
    head_center()
    left_arm_home()
    right_arm_home()

# LEFT

def left_claw_open_half():
    print("-----> left_claw_open")
    pub_left_arm_claw.publish(0.8)

def left_claw_close():
    print("-----> left_claw_close")
    pub_left_arm_claw.publish(1.5)

def left_arm_home():
    print("-----> left_arm_home")
    pub_left_arm_shoulder_lift.publish(0.25)
    pub_left_arm_elbow_rotate.publish(0.0)
    pub_left_arm_elbow_bend.publish(2.0)
    pub_left_arm_wrist_rotate.publish(0.0)
    # pub_left_arm_claw.publish(0.25)
    pub_left_arm_shoulder_rotate.publish(-1.0)

def left_arm_extend():    
    print("-----> left_arm_extend")
    pub_sidetilt.publish(0.0)
    pub_tilt.publish(0.25)
    pub_pan.publish(0.3)
    pub_left_arm_shoulder_lift.publish(0.25)
    pub_left_arm_elbow_rotate.publish(0.3)
    pub_left_arm_elbow_bend.publish(1.8)
    pub_left_arm_wrist_rotate.publish(0.0)
    #pub_left_arm_claw.publish(0.8)
    pub_left_arm_shoulder_rotate.publish(1.0)

# ========================================================================================
# Main

if __name__ == '__main__':
    pub_pan = rospy.Publisher('/head_pan_controller/command', Float64, queue_size=1)
    pub_tilt = rospy.Publisher('/head_tilt_controller/command', Float64, queue_size=1)
    pub_sidetilt = rospy.Publisher('/head_sidetilt_controller/command', Float64, queue_size=1)

    pub_left_arm_shoulder_lift.= rospy.Publisher('/left_arm_shoulder_rotate.lift_controller/command', Float64, queue_size=1)
    pub_left_arm_elbow_rotate = rospy.Publisher('/left_arm_elbow_rotate_controller/command', Float64, queue_size=1)
    pub_left_arm_elbow_bend = rospy.Publisher('/left_arm_elbow_bend_controller/command', Float64, queue_size=1)
    pub_left_arm_wrist_rotate = rospy.Publisher('/left_arm_wrist_rotate_controller/command', Float64, queue_size=1)
    pub_left_arm_claw = rospy.Publisher('/left_arm_claw_controller/command', Float64, queue_size=1)
    pub_left_arm_shoulder_rotate.= rospy.Publisher('/left_arm_shoulder_rotate.rotate_controller/command', Float64, queue_size=1)

    pub_right_arm_shoulder_lift.= rospy.Publisher('/right_arm_shoulder_rotate.lift_controller/command', Float64, queue_size=1)
    pub_right_arm_elbow_rotate = rospy.Publisher('/right_arm_elbow_rotate_controller/command', Float64, queue_size=1)
    pub_right_arm_elbow_bend = rospy.Publisher('/right_arm_elbow_bend_controller/command', Float64, queue_size=1)
    pub_right_arm_wrist_rotate = rospy.Publisher('/right_arm_wrist_rotate_controller/command', Float64, queue_size=1)
    pub_right_arm_claw = rospy.Publisher('/right_arm_claw_controller/command', Float64, queue_size=1)
    pub_right_arm_shoulder_rotate.= rospy.Publisher('/right_arm_shoulder_rotate.rotate_controller/command', Float64, queue_size=1)



# Initialize node for ROS
rospy.init_node('listener', anonymous=True) # TODO change this!

# Wait for Keyboard command
print();
print("Take item...")


##### START #####
left_claw_open_half()
left_arm_extend()
key = raw_input("Insert Pen, then Enter")
left_claw_close()
time.sleep(2.0)
left_arm_home()
head_center()
key = raw_input("Enter TWICE to Release Pen ")
key = raw_input("Enter again to Release Pen ")
left_claw_open_half()




