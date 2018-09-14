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
from std_msgs.msg import Float64
from std_msgs.msg import UInt16
from std_msgs.msg import UInt32
from std_msgs.msg import Empty

# for talking
import actionlib
import actionlib.action_client
import audio_and_speech_common.msg

# for servos
#from sheldon_servos.head_servo_publishers import *
#from sheldon_servos.right_arm_servo_publishers import *
#from sheldon_servos.left_arm_servo_publishers import *

from sheldon_servos.standard_servo_positions import *
from sheldon_servos.set_servo_speed import *
from sheldon_servos.set_servo_torque import *



class BehaviorAction(object):
    _feedback = behavior_common.msg.behaviorFeedback()
    _result = behavior_common.msg.behaviorResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, behavior_common.msg.behaviorAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        rospy.loginfo('%s: Initializing Wakeup behavior service' % (self._action_name))
      
    def execute_cb(self, goal):
        rospy.loginfo('%s: Executing behavior' % (self._action_name))
        rospy.loginfo( "Param1: '%s'", goal.param1)
        rospy.loginfo( "Param2: '%s'", goal.param2)

        # ====== Behavior Implementation ======  
        success = True

        pub_waist = rospy.Publisher('/waist_calibrate', Empty, queue_size=2)        
        pub_eye_cmd = rospy.Publisher('/head/eye_cmd', UInt16, queue_size=10)        
        pub_eye_color = rospy.Publisher('/head/eye_color', UInt32, queue_size=10)        
        pub_ear_cmd = rospy.Publisher('/head/ear_cmd', UInt16, queue_size=10)        
        pub_light_mode = rospy.Publisher('/arm_led_mode', UInt16, queue_size=10)        

        rospy.loginfo("Waiting for speech server (press ctrl-c to cancel at anytime)")
        client = actionlib.SimpleActionClient("/speech_service", audio_and_speech_common.msg.speechAction)
        client.wait_for_server()

        pub_eye_cmd.publish(2) # 2 = Turn eyes on, normal blink mode
        pub_eye_color.publish(0x00002f) # Default Blue
        pub_ear_cmd.publish(1) # 2 = Turn ear lights on, normal mode
        pub_light_mode.publish(0) # 0 = lights off by default

        rospy.loginfo("Talking...")
        goal = audio_and_speech_common.msg.speechGoal(text_to_speak="initializing system")
        client.send_goal(goal)
        result = client.wait_for_result() # wait for speech to complete
        rospy.loginfo("Speech goal returned result: %d", result)

        goal = audio_and_speech_common.msg.speechGoal(text_to_speak="clear objects close to me")
        client.send_goal(goal)
        result = client.wait_for_result() # wait for speech to complete
        rospy.loginfo("Speech goal returned result: %d", result)

        # Move head and arms to ready position
        SetServoTorque(0.8, all_joints)
        SetServoSpeed(0.8, all_joints)
        all_home()

        #calibrate waist
        rospy.loginfo('  calibrating waist Position...')
        pub_waist.publish()


        time.sleep(5.0) # seconds
        rospy.loginfo('  Initialization Complete.')
        rospy.loginfo("Talking...")
        goal = audio_and_speech_common.msg.speechGoal(text_to_speak="all systems ready")
        client.send_goal(goal)
        result = client.wait_for_result()
        rospy.loginfo("Speech goal returned result: %d", result)

          
        if success:
            rospy.loginfo('%s: Behavior complete' % self._action_name)
            self._as.set_succeeded(self._result)

        
if __name__ == '__main__':
    rospy.init_node('wakeup_behavior')
    server = BehaviorAction(rospy.get_name())
    rospy.spin()
