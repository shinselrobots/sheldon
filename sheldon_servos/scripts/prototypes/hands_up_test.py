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
#from dynamixel_controllers.srv import TorqueEnable, SetTorqueLimit, SetSpeed
#from sheldon_servos.head_servo_publishers import *
#from sheldon_servos.right_arm_servo_publishers import *
#from sheldon_servos.left_arm_servo_publishers import *
#from sheldon_servos.servo_joint_list import all_joints, head_joints, right_arm_joints

from sheldon_servos.standard_servo_positions import *
from sheldon_servos.set_servo_speed import *
from sheldon_servos.set_servo_torque import *

#from sheldon_servos.dynamixel_relax_all_servos import *
#from sheldon_servos.dynamixel_joint_state_publisher import *

# 90 degrees = 1.57, 45 = 0.785

# global

def right_arm_test0():
    pub_right_arm_shoulder_rotate.publish(0.5)

def left_arm_test0():
    pub_left_arm_shoulder_rotate.publish(-0.5)

def right_arm_test1():
    pub_right_arm_shoulder_rotate.publish(-4.0)

def left_arm_test1():
    pub_left_arm_shoulder_rotate.publish(4.0)

def right_arm_up1():
    #print("-----> right_arm_home")
    pub_right_arm_shoulder_rotate.publish(-3.0)

def left_arm_up1():
    #print("-----> left_arm_home")
    pub_left_arm_shoulder_rotate.publish(3.0)

def right_arm_up2():
    #print("-----> right_arm_home")
    #pub_right_arm_shoulder_rotate.publish(-3.0)
    #pub_right_arm_shoulder_lift.publish(0.5)
    #pub_right_arm_elbow_rotate.publish(-1.5)
    pub_right_arm_elbow_bend.publish(0.5)
    pub_right_arm_wrist_rotate.publish(0.0)
    #pub_right_arm_gripper.publish(-2.0)


def left_arm_up2():
    #print("-----> left_arm_home")
    #pub_left_arm_shoulder_rotate.publish(3.0)
    #pub_left_arm_shoulder_lift.publish(0.7)
    #pub_left_arm_elbow_rotate.publish(1.5)
    pub_left_arm_elbow_bend.publish(0.5)
    pub_left_arm_wrist_rotate.publish(0.0)
    # pub_left_arm_gripper.publish(0.25)

def say_something():

  talkString = "I am not the droid you are looking for"
  rospy.loginfo("Talking")
  goal = audio_and_speech_common.msg.speechGoal(text_to_speak=talkString)
  client.send_goal(goal)

  result = client.wait_for_result()
  rospy.loginfo("Behavior returned result: %d", result)


# ========================================================================================
# Main

if __name__ == '__main__':


  # Initialize node for ROS
  rospy.init_node('hands_up', anonymous=True) # TODO change this!

  rospy.loginfo("Waiting for speech server (press ctrl-c to cancel at anytime)")
  client = actionlib.SimpleActionClient("/speech_service", audio_and_speech_common.msg.speechAction)
  client.wait_for_server()

  SetServoTorque(0.5, right_arm_joints)
  SetServoSpeed(0.5, right_arm_joints)
  SetServoTorque(0.5, left_arm_joints)
  SetServoSpeed(0.5, left_arm_joints)

  SetSingleServoSpeed(1.5, 'right_arm_shoulder_rotate_joint')
  SetSingleServoSpeed(1.5, 'left_arm_shoulder_rotate_joint')

  all_home()
  #time.sleep(3)
  head_center()
  right_arm_up1()
  left_arm_up1()
  time.sleep(1)

  right_arm_up2()
  left_arm_up2()
  time.sleep(3)

  SetSingleServoSpeed(0.5, 'right_arm_shoulder_rotate_joint')
  SetSingleServoSpeed(0.5, 'left_arm_shoulder_rotate_joint')

  time.sleep(2)
  SetSingleServoSpeed(1.5, 'right_arm_shoulder_rotate_joint')
  SetSingleServoSpeed(1.5, 'left_arm_shoulder_rotate_joint')


  #say_something()
  all_home()
  #right_arm_test0()
  #left_arm_test0()





