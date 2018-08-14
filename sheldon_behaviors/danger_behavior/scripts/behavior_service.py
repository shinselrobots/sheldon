#! /usr/bin/env python
# License: Apache 2.0. See LICENSE file in root directory.
#
# For simple behaviors that can run syncronously, Python provides
# a simple way to implement this.  Add the work of your behavior
# in the execute_cb callback
#

import rospy
import actionlib
import behavior_common.msg

import time
import random
import math
from std_msgs.msg import Float64
from std_msgs.msg import Empty

# for talking
import actionlib
import actionlib.action_client
import audio_and_speech_common.msg

# for sound effects
import os, thread
from playsound import playsound

# for servos
#from sheldon_servos.head_servo_publishers import *
#from sheldon_servos.right_arm_servo_publishers import *
#from sheldon_servos.left_arm_servo_publishers import *

from sheldon_servos.standard_servo_positions import *
from sheldon_servos.set_servo_speed import *
from sheldon_servos.set_servo_torque import *

# Globals

interrupted = False
m_2PI = (2 * math.pi)

def danger_position_1():
    pub_right_arm_shoulder_rotate.publish(3.0)
    pub_left_arm_shoulder_rotate.publish(2.5)

def danger_position_2():
    pub_right_arm_shoulder_rotate.publish(3.0)
    #pub_right_arm_shoulder_lift.publish(0.5)
    #pub_right_arm_elbow_rotate.publish(-1.5)
    pub_right_arm_elbow_bend.publish(0.5) 
    #pub_right_arm_wrist_rotate.publish(0.0)
    #pub_right_arm_gripper.publish(0.0)

    pub_left_arm_shoulder_rotate.publish(2.5)
    #pub_left_arm_shoulder_lift.publish(0.7)
    #pub_left_arm_elbow_rotate.publish(1.5)
    pub_left_arm_elbow_bend.publish(1.5)
    pub_left_arm_wrist_rotate.publish(0.0)
    # pub_left_arm_gripper.publish(0.0)
    head_home() 


def danger_position_3():
    pub_right_arm_shoulder_rotate.publish(2.5)
    #pub_right_arm_shoulder_lift.publish(0.5)
    #pub_right_arm_elbow_rotate.publish(-1.5)
    pub_right_arm_elbow_bend.publish(1.5)
    #pub_right_arm_wrist_rotate.publish(0.0)
    #pub_right_arm_gripper.publish(0.0)

    pub_left_arm_shoulder_rotate.publish(3.0)
    #pub_left_arm_shoulder_lift.publish(0.7)
    #pub_left_arm_elbow_rotate.publish(1.5)
    pub_left_arm_elbow_bend.publish(0.5)
    #pub_left_arm_wrist_rotate.publish(0.0)
    # pub_left_arm_gripper.publish(0.0)
    head_home() 



class BehaviorAction(object):
    _feedback = behavior_common.msg.behaviorFeedback()
    _result = behavior_common.msg.behaviorResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, behavior_common.msg.behaviorAction, execute_cb=self.execute_cb, auto_start = False)
        rospy.loginfo('%s: Initializing Danger behavior service' % (self._action_name))

        self.resource_dir = rospy.get_param('resource_dir', 
          "/home/system/catkin_robot/src/sheldon/sheldon_behaviors/resources/sounds")
        self.sound_effect_danger = os.path.join(self.resource_dir, "danger_will_roibinson.wav")
        rospy.loginfo("DBG: self.sound_effect_danger: %s", self.sound_effect_danger)
        self._as.start()

    def InterruptRequested(self):
        # check that preempt has not been requested by the client
        if self._as.is_preempt_requested():
            rospy.logwarn('=====>  %s: BEHAVIOR PREEMPTED!' % self._action_name)
            interrupted = True
        else:
            interrupted = False
        return interrupted

    def stop_turn(self):
        # stop any wheel motion
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        self.pub_wheel_motors.publish(twist)

    def cleanup(self):
        # clean everyting up before exiting
        self.stop_turn()

        all_home() # arms/head in home position

        # un-mute the microphone
        self.mic_system_enable_pub.publish(True)

        if interrupted:
            rospy.loginfo('%s: Behavior preempted' % self._action_name)
            self._as.set_preempted()
        else:
            rospy.loginfo('%s: Behavior complete' % self._action_name)
            self._as.set_succeeded(self._result)

    def speech_done_cb(self, goal_status, result):
        rospy.loginfo("got SPEECH DONE message")
        self.speech_done = True

    def execute_cb(self, goal):
        rospy.loginfo('%s: Executing behavior' % (self._action_name))
        rospy.loginfo( "Param1: '%s'", goal.param1)
        rospy.loginfo( "Param2: '%s'", goal.param2)

        # ====== Behavior Implementation ======  

        # Initialization
        interrupted = False
        self.compass = float('nan')
        self.start_position = float('nan')
        self.speech_done = False
        #r = rospy.Rate(1.0)

        rospy.loginfo('%s: Running behavior' % (self._action_name))
        self._feedback.running = True
        self._as.publish_feedback(self._feedback)

        # Publish wheel motor commands as low priority to motor control
        # TODO: Put this in launch file: 
        #   <!-- <remap from="cmd_vel" to="move_base/priority1"/>  --> 
        #self.pub_wheel_motors = rospy.Publisher('~cmd_vel', Twist, queue_size=5)
        self.pub_wheel_motors = rospy.Publisher('move_base/priority1', Twist, queue_size=5)

        # compass_sub_ = rospy.Subscriber('/compass', Float32, self.compass_cb) 

        rospy.loginfo("Waiting for speech server (press ctrl-c to cancel at anytime)")
        self.speech_client = actionlib.SimpleActionClient("/speech_service",
            audio_and_speech_common.msg.speechAction)
        self.speech_client.wait_for_server()

        SetServoTorque(0.5, all_joints)
        SetServoSpeed(0.5, all_joints)
        SetSingleServoSpeed(2.0, 'right_arm_shoulder_rotate_joint')
        SetSingleServoSpeed(2.0, 'left_arm_shoulder_rotate_joint')

        SetSingleServoSpeed(2.0, 'right_arm_elbow_bend_joint')
        SetSingleServoSpeed(2.0, 'left_arm_elbow_bend_joint')

        # mute the microphone, so the robot does not hear sounds and servos!
        self.mic_system_enable_pub.publish(False)

        # =======================================================================
        # start by putting both arms up into first position 
        danger_position_1()

        # play wave file "Danger, Will Robinson!" (blocking)
        playsound(self.sound_effect_danger)

        # Publish motor command TODO
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 1.0 # SPEED OF TURN
        # self.pub_wheel_motors.publish(twist)

        time.sleep(1) # give time for arms to get up

        # start saying phrase
        rospy.loginfo("Talking")
        goal = audio_and_speech_common.msg.speechGoal(text_to_speak=
            "When in danger when in doubt run in circles scream and shout, when in danger when in doubt run in circles scream and shout.")
        self.speech_client.send_goal(goal, self.speech_done_cb)
        # don't wait for speech to complete!

        # Move arms to various positions as the robot spins
        # TODO - blink body lights

        for i in range(0, 3):
            danger_position_2()
            if self.InterruptRequested():
                break
            check_turn_complete()
            rospy.sleep(1.0)

            danger_position_3()
            if self.InterruptRequested():
                break
            check_turn_complete()
            rospy.sleep(1.0)

        # Move head and arms back to ready position
        #time.sleep(1)
        #SetSingleServoSpeed(1.5, 'right_arm_shoulder_rotate_joint')
        #SetSingleServoSpeed(1.5, 'left_arm_shoulder_rotate_joint')
        all_home() # arms/head in home position
        self.stop_turn()

        if not self.InterruptRequested():
 
            goal = audio_and_speech_common.msg.speechGoal(text_to_speak=
                "Woah, what just happened? I think I blew a circuit")
            speech_client.send_goal(goal)
            result = speech_client.wait_for_result()   # wait for speech to complete
            rospy.loginfo("Speech goal returned result: %d", result)

        # Finish Behavior
        cleanup()
 
        
if __name__ == '__main__':
    rospy.init_node('danger_behavior')
    server = BehaviorAction(rospy.get_name())
    rospy.spin()
