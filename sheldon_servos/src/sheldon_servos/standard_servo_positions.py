#!/usr/bin/env python
# 90 degrees = 1.57, 45 = 0.785

#import roslib
#roslib.load_manifest('sheldon_servos')
import rospy
#import time
from std_msgs.msg import Float64
from sheldon_servos.head_servo_publishers import *
from sheldon_servos.right_arm_servo_publishers import *
from sheldon_servos.left_arm_servo_publishers import *
from sheldon_servos.waist_publishers import *


def all_home(): # all in home position
    print("-----> all_home")
    head_home()
    left_arm_home()
    right_arm_home()

def all_sleep(): # all in sleep position
    print("-----> all_sleep")
    head_sleep()
    left_arm_sleep()
    right_arm_sleep()


# WAIST

def waist_bow_down():
    print("-----> waist bow down")
    pub_waist_position.publish(0.8) # Bow Down

def waist_home():
    print("-----> waist bow down")
    pub_waist_position.publish(0.0) # home

def waist_full_down():
    print("-----> waist todo full down")
    pub_waist_position.publish(1.0) # MAX is 58 degrees = 1.0122 radians

# HEAD

def head_center():
    print("-----> head_center")
    pub_head_sidetilt.publish(0.0)
    pub_head_tilt.publish(-0.3) # Tilt up a bit
    pub_head_pan.publish(0.0)

def head_home():
    print("-----> head_home")
    pub_head_sidetilt.publish(0.0)
    pub_head_tilt.publish(-0.3) # Tilt up a bit
    pub_head_pan.publish(0.0)

def head_sleep():
    print("-----> head_sleep")
    pub_head_sidetilt.publish(0.0)
    pub_head_tilt.publish(1.26)
    pub_head_pan.publish(0.0)


# RIGHT ARM

def right_gripper_open_half():
    print("-----> right_gripper_open_half")
    pub_right_arm_gripper_finger.publish(1.0)

def right_gripper_open_full():
    print("-----> right_gripper_open_full")
    pub_right_arm_gripper_finger.publish(3.1)

def right_gripper_close():
    print("-----> right_gripper_close")
    pub_right_arm_gripper_finger.publish(0.0)

def right_gripper_close_tight():
    print("-----> right_gripper_close_tight")
    pub_right_arm_gripper_finger.publish(-0.1)

def right_arm_home():
    print("-----> right_arm_home")
    pub_right_arm_shoulder_rotate.publish(-0.5) 
    pub_right_arm_shoulder_lift.publish(0.0)
    pub_right_arm_elbow_rotate.publish(0.0)
    pub_right_arm_elbow_bend.publish(2.2)
    pub_right_arm_wrist_bend.publish(0.0)
    pub_right_arm_wrist_rotate.publish(0.0)
    #pub_right_arm_gripper_finger.publish(0.0)

def right_arm_extend():
    print("-----> right_arm_extend")
    pub_head_sidetilt.publish(0.0)
    pub_head_tilt.publish(0.25)
    pub_head_pan.publish(-0.3)

    pub_right_arm_shoulder_rotate.publish(1.0)  
    pub_right_arm_shoulder_lift.publish(0.1)
    pub_right_arm_elbow_rotate.publish(-0.25)
    pub_right_arm_elbow_bend.publish(1.8)
    pub_right_arm_wrist_bend.publish(0.0)
    pub_right_arm_wrist_rotate.publish(0.0)
    #pub_right_arm_gripper_finger.publish(0.0)

def right_arm_sleep():
    print("-----> right_arm_sleep")
    pub_right_arm_shoulder_rotate.publish(0.2) 
    pub_right_arm_shoulder_lift.publish(0.1)
    pub_right_arm_elbow_rotate.publish(0.07)
    pub_right_arm_elbow_bend.publish(3.00)  # 3.13
    pub_right_arm_wrist_bend.publish(0.18)   # 0.0
    pub_right_arm_wrist_rotate.publish(0.14) 
    pub_right_arm_gripper_finger.publish(0.0)

def right_arm_down():
    print("-----> right_arm_down")
    pub_right_arm_shoulder_rotate.publish(0.0)  
    pub_right_arm_shoulder_lift.publish(0.1)
    pub_right_arm_elbow_rotate.publish(0.0)
    pub_right_arm_elbow_bend.publish(0.0)
    pub_right_arm_wrist_bend.publish(0.0)
    pub_right_arm_wrist_rotate.publish(0.0)
    pub_right_arm_gripper_finger.publish(0.0)

# LEFT ARM

def left_gripper_open_half():
    print("-----> left_gripper_open_half")
    pub_left_arm_gripper_finger.publish(1.0)

def left_gripper_open_full():
    print("-----> left_gripper_open_full")
    pub_left_arm_gripper_finger.publish(3.1)

def left_gripper_close():
    print("-----> left_gripper_close")
    pub_left_arm_gripper_finger.publish(0.0)

def left_gripper_close_tight():
    print("-----> left_gripper_close_tight")
    pub_left_arm_gripper_finger.publish(-0.1)

def left_arm_home():
    print("-----> left_arm_home")
    pub_left_arm_shoulder_rotate.publish(-0.5) 
    pub_left_arm_shoulder_lift.publish(0.0)
    pub_left_arm_elbow_rotate.publish(0.0)
    pub_left_arm_elbow_bend.publish(2.2)
    pub_left_arm_wrist_bend.publish(0.0)
    pub_left_arm_wrist_rotate.publish(0.0)
    # pub_left_arm_gripper_finger.publish(0.0)

def left_arm_extend():    
    print("-----> left_arm_extend")
    pub_head_sidetilt.publish(0.0)
    pub_head_tilt.publish(0.25)
    pub_head_pan.publish(0.3)

    pub_left_arm_shoulder_rotate.publish(1.0)
    pub_left_arm_shoulder_lift.publish(0.1)
    pub_left_arm_elbow_rotate.publish(0.3)
    pub_left_arm_elbow_bend.publish(1.8)
    pub_left_arm_wrist_bend.publish(0.0)
    pub_left_arm_wrist_rotate.publish(0.0)
    # pub_left_arm_gripper_finger.publish(0.0)

def left_arm_down():    
    print("-----> left_arm_down")
    pub_left_arm_shoulder_rotate.publish(0.0)
    pub_left_arm_shoulder_lift.publish(0.1)
    pub_left_arm_elbow_rotate.publish(0.0)
    pub_left_arm_elbow_bend.publish(0.0)
    pub_left_arm_wrist_bend.publish(0.0)
    pub_left_arm_wrist_rotate.publish(0.0)
    pub_left_arm_gripper_finger.publish(0.0)

def left_arm_sleep():    
    print("-----> left_arm_sleep")
    pub_left_arm_shoulder_rotate.publish(0.2)
    pub_left_arm_shoulder_lift.publish(0.1)
    pub_left_arm_elbow_rotate.publish(-0.02) # align with lock
    pub_left_arm_elbow_bend.publish(3.13)   #3.0
    pub_left_arm_wrist_bend.publish(0.06)     
    pub_left_arm_wrist_rotate.publish(-0.1)
    pub_left_arm_gripper_finger.publish(0.0)


    #pub_right_arm_elbow_rotate.publish(0.07)
    #pub_right_arm_elbow_bend.publish(3.00)  # 3.13
    #pub_right_arm_wrist_bend.publish(0.18)   # 0.0
    #pub_right_arm_wrist_rotate.publish(0.14) 




