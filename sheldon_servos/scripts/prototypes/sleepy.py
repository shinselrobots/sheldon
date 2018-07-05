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

  say_something()

  SetTorque(0.8, all_joints)
  SetServoSpeed(0.5, all_joints)

  all_sleep()
  time.sleep(5)

  SetTorque(0.0, all_joints)
#  SetTorque(0.0, right_arm_joints)
#  SetTorque(0.0, left_arm_joints)

  time.sleep(5)


