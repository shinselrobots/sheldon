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
from std_msgs.msg import Bool
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

# Globals

# TODO! FIX THESE BOGUS VALUES!!!
def move_arm_shake_preliminary():
    pub_right_arm_shoulder_rotate.publish(-1.12)
    pub_right_arm_shoulder_lift.publish(0.05)
    head_home() 

def move_arm_shake_ready():
    pub_right_arm_shoulder_rotate.publish(-1.12)
    pub_right_arm_shoulder_lift.publish(0.05)
    pub_right_arm_elbow_rotate.publish(-0.47)
    pub_right_arm_elbow_bend.publish(1.37)
    pub_right_arm_wrist_rotate.publish(-1.35)
    pub_right_arm_claw.publish(-1.675)

def move_arm_shake_up():
    pub_right_arm_elbow_bend.publish(1.47)
    #head_home() 

def move_arm_shake_down():
    pub_right_arm_elbow_bend.publish(1.09)
    #head_home() 



class BehaviorAction(object):
    _feedback = behavior_common.msg.behaviorFeedback()
    _result = behavior_common.msg.behaviorResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, behavior_common.msg.behaviorAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

        # enable/disable microphone when robot is moving servos.  
        # (Note system_enable vs. speech_enable vs. user_enable)
        self.mic_system_enable_pub = rospy.Publisher('/microphone/system_enable', Bool, queue_size=1)        

        sensor_sub = rospy.Subscriber('/arm_hand_sensor_right', UInt16, self.sensor_cb) # hand sensor messages

        rospy.loginfo('%s: Initialized behavior service' % (self._action_name))


    def sensor_cb(self, msg):
        # rospy.loginfo("%s: sensor_cb called", self._action_name)
        nearest_object_to_hand = msg.data
        if nearest_object_to_hand < 100: # mm
            #rospy.loginfo("%s: sensor_cb: Hand detected", self._action_name)
            self.hand_detected = True
        else:
            self.hand_detected = False

    # ===========================================================================
    # Utility functions

    def InterruptRequested(self):
        # check if preempt has been requested by the client
        if self._as.is_preempt_requested():
            rospy.logwarn('=====>  %s: BEHAVIOR PREEMPTED!' % self._action_name)
            self.cleanup(True)
            return True
        else:
            return False

    def cleanup(self, interrupted):
        # clean everyting up before exiting

        # Restore Servo defaults
        SetServoTorque(0.5, all_joints)
        SetServoSpeed(0.5, all_joints)

        # Move head and arms to ready position
        all_home()

        # allow time for servos to complete moving
        time.sleep(2) 

        # un-mute the microphone
        self.mic_system_enable_pub.publish(True)

        if interrupted:
            rospy.loginfo('%s: Behavior preempted' % self._action_name)
            self._as.set_preempted()
        else:
            rospy.loginfo('%s: Behavior complete' % self._action_name)
            self._as.set_succeeded(self._result)


    #====================================================================
    # Execute Behavior
    def execute_cb(self, goal):
        rospy.loginfo('%s: Executing behavior' % (self._action_name))
        rospy.loginfo( "Param1: '%s'", goal.param1)
        rospy.loginfo( "Param2: '%s'", goal.param2)

        # ====== Behavior Implementation ======  
        success = True
        r = rospy.Rate(1.0)

        # initialization
        rospy.loginfo("Waiting for speech server (press ctrl-c to cancel at anytime)")
        client = actionlib.SimpleActionClient("/speech_service", audio_and_speech_common.msg.speechAction)
        client.wait_for_server()

        SetServoTorque(0.5, all_joints) # NOTE Extra weak Servos!
        SetServoSpeed(0.5, all_joints)
        SetSingleServoSpeed(1.5, 'right_arm_shoulder_rotate_controller')
        #SetSingleServoSpeed(1.5, 'left_arm_shoulder_rotate_controller')

        # mute the microphone
        self.mic_system_enable_pub.publish(False)
 
        # Move arm into shake position
        move_arm_shake_preliminary()
        time.sleep(2)
        move_arm_shake_ready()

        # say hello while arms are moving
        rospy.loginfo("Talking")
        goal = audio_and_speech_common.msg.speechGoal(text_to_speak="hello")
        client.send_goal(goal)
        result = client.wait_for_result()   # wait for speech to complete
        rospy.loginfo("Speech goal returned result: %d", result)

        # wait for sensor reading, indicating person is holding hand
        for i in range (1,26): # note, about 6 seconds of this time the arm is still moving into position
            if self.InterruptRequested():
                return
            if self.hand_detected:
                rospy.loginfo("DBG: hand detected")
                break
            time.sleep(0.5) 

        if not self.hand_detected:
            # no person detected
            rospy.loginfo("Talking")
            goal = audio_and_speech_common.msg.speechGoal(text_to_speak="well, this is awkward")
            client.send_goal(goal)
            result = client.wait_for_result()   # wait for speech to complete
            rospy.loginfo("Speech goal returned result: %d", result)
            self.cleanup(False)
            return

        # say something nice
        rospy.loginfo("Talking")
        goal = audio_and_speech_common.msg.speechGoal(text_to_speak="I am pleased to meet you")
        client.send_goal(goal)
        # Don't wait for speech to complete, start shaking hands
        #result = client.wait_for_result()   # wait for speech to complete
        #rospy.loginfo("Speech goal returned result: %d", result)
        time.sleep(2)

        # Start shaking hands. 
        for i in range(1, 4):

            move_arm_shake_up() # 1
            if self.InterruptRequested():
                return
            time.sleep(1)

            move_arm_shake_down()
            if self.InterruptRequested():
                return
            time.sleep(1)

            if not self.hand_detected:
                # no person detected, they must have let go
                rospy.loginfo("DBG: person let go early")
                self.cleanup(False)
                return

        # done shaking hands, wait for user to let go
        for i in range (1,6):
            if self.InterruptRequested():
                return
            if not self.hand_detected: # user let go
                rospy.loginfo("DBG: person let go at appropriate time")
                break
            time.sleep(0.5) 

        if self.hand_detected:
            # person still holding hand
            rospy.loginfo("Talking")
            goal = audio_and_speech_common.msg.speechGoal(text_to_speak="um, you can let go now")
            client.send_goal(goal)
            result = client.wait_for_result()   # wait for speech to complete
            rospy.loginfo("Speech goal returned result: %d", result)
 

       # wait for user to let go
        for i in range (1,10):
            if self.InterruptRequested():
                return
            if not self.hand_detected:
                rospy.loginfo("DBG: person finally let go")
                break
            time.sleep(0.5) 


        # Behavior Done, clean up
        # Includes moving head and arms back to ready position
        self.cleanup(False)
 
        
if __name__ == '__main__':
    rospy.init_node('shake_hands_behavior')
    server = BehaviorAction(rospy.get_name())
    rospy.spin()
