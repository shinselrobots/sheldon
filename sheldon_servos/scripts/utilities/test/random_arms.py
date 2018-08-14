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

def right_arm_home():
    print("-----> right_arm_home")
    #pub_right_arm_shoulder_lift.publish(0.25)
    pub_right_arm_elbow_rotate.publish(0.0)
    pub_right_arm_elbow_bend.publish(2.0)
    pub_right_arm_wrist_rotate.publish(0.0)
    pub_right_arm_gripper.publish(0.25)
    pub_right_arm_shoulder_rotate.publish(-1.0)

def left_arm_home():
    print("-----> left_arm_home")
    pub_left_arm_shoulder_lift.publish(0.25)
    pub_left_arm_elbow_rotate.publish(0.0)
    pub_left_arm_elbow_bend.publish(2.0)
    pub_left_arm_wrist_rotate.publish(0.0)
    # pub_left_arm_gripper.publish(0.25)
    pub_left_arm_shoulder_rotate.publish(-1.0)




def move_random( arm ):
    elbow_rotate = random.uniform(-0.05, 0.05)
    elbow_bend = random.uniform(-0.05, 0.05)
    shoulder = random.uniform(-0.05, 0.05)

    if( arm == 'right' ):
        #pub_right_arm_shoulder_lift.publish(0.25)
        pub_right_arm_elbow_rotate.publish(0.0 + elbow_rotate)
        pub_right_arm_elbow_bend.publish(2.0 + elbow_bend)
        pub_right_arm_shoulder_rotate.publish(-1.0 + shoulder)

    else:
        pub_left_arm_elbow_rotate.publish(0.0 + elbow_rotate)
        pub_left_arm_elbow_bend.publish(2.0 + elbow_bend)
        pub_left_arm_shoulder_rotate.publish(-1.0 + shoulder)


# ========================================================================================
# Main

if __name__ == '__main__':
    pub_pan = rospy.Publisher('/head_pan_joint/command', Float64, queue_size=1)
    pub_tilt = rospy.Publisher('/head_tilt_joint/command', Float64, queue_size=1)
    pub_sidetilt = rospy.Publisher('/head_sidetilt_joint/command', Float64, queue_size=1)

    pub_left_arm_shoulder_lift.= rospy.Publisher('/left_arm_shoulder_rotate.lift_joint/command', Float64, queue_size=1)
    pub_left_arm_elbow_rotate = rospy.Publisher('/left_arm_elbow_rotate_joint/command', Float64, queue_size=1)
    pub_left_arm_elbow_bend = rospy.Publisher('/left_arm_elbow_bend_joint/command', Float64, queue_size=1)
    pub_left_arm_wrist_rotate = rospy.Publisher('/left_arm_wrist_rotate_joint/command', Float64, queue_size=1)
    pub_left_arm_gripper = rospy.Publisher('/left_arm_gripper_controller/command', Float64, queue_size=1)
    pub_left_arm_shoulder_rotate.= rospy.Publisher('/left_arm_shoulder_rotate.rotate_joint/command', Float64, queue_size=1)

    pub_right_arm_shoulder_lift.= rospy.Publisher('/right_arm_shoulder_rotate.lift_joint/command', Float64, queue_size=1)
    pub_right_arm_elbow_rotate = rospy.Publisher('/right_arm_elbow_rotate_joint/command', Float64, queue_size=1)
    pub_right_arm_elbow_bend = rospy.Publisher('/right_arm_elbow_bend_joint/command', Float64, queue_size=1)
    pub_right_arm_wrist_rotate = rospy.Publisher('/right_arm_wrist_rotate_joint/command', Float64, queue_size=1)
    pub_right_arm_gripper = rospy.Publisher('/right_arm_gripper_controller/command', Float64, queue_size=1)
    pub_right_arm_shoulder_rotate.= rospy.Publisher('/right_arm_shoulder_rotate.rotate_joint/command', Float64, queue_size=1)



# Initialize node for ROS
rospy.init_node('listener', anonymous=True) # TODO change this!

# Wait for Keyboard command
print();
print("Random arm movement - press Ctrl-Z to exit")

right_arm_home()
left_arm_home()

while(1):
    move_random( 'right' )
    ranTime = 1.0 #random.uniform(0.8, 2.5)
    time.sleep(ranTime)


