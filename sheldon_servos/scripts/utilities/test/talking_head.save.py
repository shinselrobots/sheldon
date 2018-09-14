#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Float64
import random

# for talking
import actionlib
import actionlib.action_client
import audio_and_speech_common.msg

# for servos
from dynamixel_controllers.srv import TorqueEnable, SetTorqueLimit, SetSpeed
from sheldon_servos.servo_joint_list import all_joints, head_joints, right_arm_joints
from sheldon_servos.servo_publishers import *
from sheldon_servos.standard_servo_positions import *
from set_servo_speed import *
from set_servo_torque import *

#from sheldon_servos.dynamixel_relax_all_servos import *
#from sheldon_servos.dynamixel_joint_state_publisher import *

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

def move_random():
    sidetiltAmt = random.uniform(-0.05, 0.05)
    tiltAmt = random.uniform(-0.3, 0.3)
    panAmt = random.uniform(-0.5, 0.5)

    pub_sidetilt.publish(sidetiltAmt)
    pub_tilt.publish(tiltAmt)
    pub_pan.publish(panAmt)


def say_something():
    rospy.loginfo("Waiting for speech server (press ctrl-c to cancel at anytime)")
    client = actionlib.SimpleActionClient("/speech_service", audio_and_speech_common.msg.speechAction)
    client.wait_for_server()

    talkString = "starting random movement"
    rospy.loginfo("Talking")
    goal = audio_and_speech_common.msg.speechGoal(text_to_speak=talkString)
    client.send_goal(goal)

    result = client.wait_for_result()
    rospy.loginfo("Behavior returned result: %d", result)



# ========================================================================================
# Main

if __name__ == '__main__':
    pub_pan = rospy.Publisher('/head_pan_joint/command', Float64, queue_size=1)
    pub_tilt = rospy.Publisher('/head_tilt_joint/command', Float64, queue_size=1)
    pub_sidetilt = rospy.Publisher('/head_sidetilt_joint/command', Float64, queue_size=1)

    pub_left_arm_shoulder_lift = rospy.Publisher('/left_arm_shoulder_lift_joint/command', Float64, queue_size=1)
    pub_left_arm_elbow_rotate = rospy.Publisher('/left_arm_elbow_rotate_joint/command', Float64, queue_size=1)
    pub_left_arm_elbow_bend = rospy.Publisher('/left_arm_elbow_bend_joint/command', Float64, queue_size=1)
    pub_left_arm_wrist_rotate = rospy.Publisher('/left_arm_wrist_rotate_joint/command', Float64, queue_size=1)
    pub_left_arm_gripper_finger = rospy.Publisher('/left_arm_gripper_finger_joint/command', Float64, queue_size=1)
    pub_left_arm_shoulder_rotate = rospy.Publisher('/left_arm_shoulder_rotate_joint/command', Float64, queue_size=1)

    pub_right_arm_shoulder_lift = rospy.Publisher('/right_arm_shoulder_lift_joint/command', Float64, queue_size=1)
    pub_right_arm_elbow_rotate = rospy.Publisher('/right_arm_elbow_rotate_joint/command', Float64, queue_size=1)
    pub_right_arm_elbow_bend = rospy.Publisher('/right_arm_elbow_bend_joint/command', Float64, queue_size=1)
    pub_right_arm_wrist_rotate = rospy.Publisher('/right_arm_wrist_rotate_joint/command', Float64, queue_size=1)
    pub_right_arm_gripper_finger = rospy.Publisher('/right_arm_gripper_finger_joint/command', Float64, queue_size=1)
    pub_right_arm_shoulder_rotate = rospy.Publisher('/right_arm_shoulder_rotate_joint/command', Float64, queue_size=1)



# Initialize node for ROS
rospy.init_node('talking_head', anonymous=True) # TODO change this!

SetTorque(0.5, head_joints)
SetServoSpeed(0.5, head_joints)

# Wait for Keyboard command
print();
print("Random head movement - press Ctrl-Z to exit")

say_something()

while(1):
    move_random()
    ranTime = random.uniform(1.0, 3.5)
    time.sleep(ranTime)


