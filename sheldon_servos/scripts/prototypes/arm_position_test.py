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

def right_arm_sl():
    print("-----> right_arm_sleep")
    pub_right_arm_shoulder_rotate.publish(0.0)
    pub_right_arm_shoulder_lift.publish(0.25)
    pub_right_arm_elbow_rotate.publish(0.07) # 0.13
    pub_right_arm_elbow_bend.publish(3.13)
    pub_right_arm_wrist_rotate.publish(0.0)
    #pub_right_arm_claw.publish(-0.8)


def say_something():
  rospy.loginfo("Waiting for speech server (press ctrl-c to cancel at anytime)")
  client = actionlib.SimpleActionClient("/speech_service", audio_and_speech_common.msg.speechAction)
  client.wait_for_server()

  talkString = "moving home"
  rospy.loginfo("Talking")
  goal = audio_and_speech_common.msg.speechGoal(text_to_speak=talkString)
  client.send_goal(goal)

  #result = client.wait_for_result()
  #rospy.loginfo("Behavior returned result: %d", result)

def wave1():
    print("-----> wave position 1")

    pub_right_arm_shoulder_rotate.publish(-1.2)
    pub_right_arm_shoulder_lift.publish(0.0)
    pub_right_arm_elbow_rotate.publish(0.0)
    pub_right_arm_elbow_bend.publish(2.0)
    pub_right_arm_wrist_rotate.publish(0.0)
    pub_right_arm_claw.publish(-0.5)

    pub_head_sidetilt.publish(0.0)
    pub_head_tilt.publish(0.0)
    pub_head_pan.publish(0.0)

def wave2():
    print("-----> wave position 2")
    #pub_head_sidetilt.publish(0.0)
    #pub_head_tilt.publish(0.0)
    #pub_head_pan.publish(0.0)

    #pub_right_arm_shoulder_rotate.publish(1.5)
    #pub_right_arm_shoulder_lift.publish(0.25)
    pub_right_arm_elbow_rotate.publish(-0.2)
    #pub_right_arm_elbow_bend.publish(1.5)
    #pub_right_arm_wrist_rotate.publish(0.0)
    #pub_right_arm_claw.publish(0.8)

def wave3():
    print("-----> wave position 3")
    #pub_head_sidetilt.publish(0.0)
    #pub_head_tilt.publish(0.0)
    #pub_head_pan.publish(0.0)
    pub_right_arm_elbow_rotate.publish(0.2)

class SetSingleServoSpeed():
    def __init__(self, speed, servo_controller):
        # input: a servo controller string, for example: 'right_arm_shoulder_rotate_controller'

        speed_service = '/' + servo_controller + '/set_speed'
        print('  waiting for service: ' + speed_service)
        rospy.wait_for_service(speed_service)  

        set_speed = rospy.ServiceProxy(speed_service, SetSpeed)
            
        # Set the speed
        print '  setting servo speed to ', speed
        set_speed(speed)

        print("Done setting speed.")

# ========================================================================================
# Main

if __name__ == '__main__':


  # Initialize node for ROS
  rospy.init_node('sleepy_head', anonymous=True) # TODO change this!

  torque = 0.2501

  SetServoTorque(0.5, right_arm_joints)
  SetServoSpeed(0.5, right_arm_joints)

  SetSingleServoSpeed(1.8, 'right_arm_shoulder_rotate_controller')

  all_home()
  time.sleep(3)

  wave1()
  time.sleep(4)
  wave2()
  time.sleep(2)
  SetSingleServoSpeed(2.0, 'right_arm_elbow_rotate_controller')
  wave3()
  time.sleep(2)
  SetSingleServoSpeed(0.5, 'right_arm_elbow_rotate_controller')
  SetSingleServoTorque(0.01, 'right_arm_elbow_rotate_controller')
  time.sleep(5)
  #say_something()
  SetServoTorque(0.5, right_arm_joints) # set back to normal
  all_home()
  pub_right_arm_claw.publish(-2.0)





