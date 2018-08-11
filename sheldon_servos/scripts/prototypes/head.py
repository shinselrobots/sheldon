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

def head_up():
    pub_head_sidetilt.publish(0.0)
    pub_head_tilt.publish(-0.3)
    pub_head_pan.publish(0.0)

def right_arm_sl():
    print("-----> right_arm_sleep")
    pub_right_arm_shoulder_rotate.publish(0.0)
    pub_right_arm_shoulder_lift.publish(0.25)
    pub_right_arm_elbow_rotate.publish(0.07) # 0.13
    pub_right_arm_elbow_bend.publish(3.13)
    pub_right_arm_wrist_rotate.publish(0.0)
    #pub_right_arm_gripper.publish(-0.8)


def say_something():
  rospy.loginfo("Waiting for speech server (press ctrl-c to cancel at anytime)")
  client = actionlib.SimpleActionClient("/speech_service", audio_and_speech_common.msg.speechAction)
  client.wait_for_server()

  talkString = "shutting down"
  rospy.loginfo("Talking")
  goal = audio_and_speech_common.msg.speechGoal(text_to_speak=talkString)
  client.send_goal(goal)

  result = client.wait_for_result()
  rospy.loginfo("Behavior returned result: %d", result)



# ========================================================================================
# Main

if __name__ == '__main__':


  # Initialize node for ROS
  rospy.init_node('sleepy_head', anonymous=True) # TODO change this!

  #say_something()

  SetServoTorque(0.8, all_joints)
  SetServoSpeed(0.3, all_joints)


  head_center()
  time.sleep(1)

  SetServoSpeed(0.5, head_joints)
  pub_head_tilt.publish(0.50)
  pub_head_pan.publish(0.0)
  pub_head_sidetilt.publish(0.0)
  time.sleep(0.9)


  SetServoSpeed(0.8, head_joints)
  pub_head_sidetilt.publish(0.2)
  time.sleep(0.4)
  pub_head_sidetilt.publish(-0.2)
  time.sleep(0.4)
  pub_head_sidetilt.publish(0.2)
  time.sleep(0.4)
  SetServoSpeed(0.5, head_joints)
  head_center()
  SetServoSpeed(0.3, head_joints)


  head_center()
  time.sleep(5)

  #SetTorque(0.0, all_joints)
  #SetTorque(0.0, right_arm_joints)
#  SetTorque(0.0, left_arm_joints)

  #time.sleep(5)


