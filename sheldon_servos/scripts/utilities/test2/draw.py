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

def left_gripper_open_half():
    print("-----> left_gripper_open")
    pub_left_arm_gripper.publish(0.8)

def left_gripper_close():
    print("-----> left_gripper_close")
    pub_left_arm_gripper.publish(1.5)

def left_arm_home():
    print("-----> left_arm_home")
    pub_left_arm_lift.publish(0.25)
    pub_left_arm_elbow_rotate.publish(0.0)
    pub_left_arm_elbow_bend.publish(2.0)
    pub_left_arm_wrist_rotate.publish(0.0)
    # pub_left_arm_gripper.publish(0.25)
    pub_left_arm_shoulder.publish(-1.0)

def left_arm_extend():    
    print("-----> left_arm_extend")
    pub_sidetilt.publish(0.0)
    pub_tilt.publish(0.25)
    pub_pan.publish(0.3)
    pub_left_arm_lift.publish(0.25)
    pub_left_arm_elbow_rotate.publish(0.3)
    pub_left_arm_elbow_bend.publish(1.8)
    pub_left_arm_wrist_rotate.publish(0.0)
    #pub_left_arm_gripper.publish(0.8)
    pub_left_arm_shoulder.publish(1.0)

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
    pub_left_arm_gripper = rospy.Publisher('/left_arm_gripper_controller/command', Float64, queue_size=1)
    pub_left_arm_shoulder = rospy.Publisher('/left_arm_shoulder_controller/command', Float64, queue_size=1)

    pub_right_arm_lift = rospy.Publisher('/right_arm_lift_joint/command', Float64, queue_size=1)
    pub_right_arm_elbow_rotate = rospy.Publisher('/right_arm_elbow_rotate_joint/command', Float64, queue_size=1)
    pub_right_arm_elbow_bend = rospy.Publisher('/right_arm_elbow_bend_joint/command', Float64, queue_size=1)
    pub_right_arm_wrist_rotate = rospy.Publisher('/right_arm_wrist_rotate_joint/command', Float64, queue_size=1)
    pub_right_arm_gripper = rospy.Publisher('/right_arm_gripper_controller/command', Float64, queue_size=1)
    pub_right_arm_shoulder = rospy.Publisher('/right_arm_shoulder_controller/command', Float64, queue_size=1)



# Initialize node for ROS
rospy.init_node('listener', anonymous=True) # TODO change this!

# Wait for Keyboard command
print();
print("draw...")


##### START #####

pub_sidetilt.publish(0.0)
pub_tilt.publish(0.5)
pub_pan.publish(0.4)

pub_left_arm_elbow_bend.publish(2.5)
key = raw_input("Enter ")

#pub_left_arm_lift.publish(0.25)
#pub_left_arm_elbow_rotate.publish(0.0)
#pub_left_arm_wrist_rotate.publish(0.0)
pub_left_arm_shoulder.publish(2.0)

key = raw_input("Enter ")  #show position
pub_left_arm_elbow_bend.publish(0.6)

key = raw_input("Enter ")


pub_left_arm_elbow_bend.publish(2.6)
time.sleep(1.7)

pub_sidetilt.publish(0.0)
pub_tilt.publish(0.0)
pub_pan.publish(1.0)


#left_arm_home()

#pub_left_arm_wrist_rotate.publish(0.0)
# pub_left_arm_gripper.publish(0.25)
pub_left_arm_shoulder.publish(-1.0)

time.sleep(1.7)
pub_left_arm_elbow_bend.publish(2.0)

#time.sleep(1.7)

#pub_sidetilt.publish(0.0)
#pub_tilt.publish(0.5)
#pub_pan.publish(0.4)


