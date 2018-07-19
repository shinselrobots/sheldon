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
def hands_up1():
    pub_right_arm_shoulder_rotate.publish(-3.0)
    pub_left_arm_shoulder_rotate.publish(3.0)

def hands_up2():
    #pub_right_arm_shoulder_rotate.publish(-3.0)
    #pub_right_arm_shoulder_lift.publish(0.5)
    #pub_right_arm_elbow_rotate.publish(-1.5)
    pub_right_arm_elbow_bend.publish(0.5)
    pub_right_arm_wrist_rotate.publish(0.0)
    #pub_right_arm_claw.publish(-2.0)

    #pub_left_arm_shoulder_rotate.publish(3.0)
    #pub_left_arm_shoulder_lift.publish(0.7)
    #pub_left_arm_elbow_rotate.publish(1.5)
    pub_left_arm_elbow_bend.publish(0.5)
    pub_left_arm_wrist_rotate.publish(0.0)
    # pub_left_arm_claw.publish(0.25)
    head_home() 


class BehaviorAction(object):
    _feedback = behavior_common.msg.behaviorFeedback()
    _result = behavior_common.msg.behaviorResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, behavior_common.msg.behaviorAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        rospy.loginfo('%s: Initializing Wave behavior service' % (self._action_name))

        # enable/disable microphone when robot is moving servos.  
        # (Note system_enable vs. speech_enable vs. user_enable)
        self.mic_system_enable_pub = rospy.Publisher('/microphone/system_enable', Bool, queue_size=1)        


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

        SetServoTorque(0.5, all_joints)
        SetServoSpeed(0.5, all_joints)
        SetSingleServoSpeed(1.5, 'right_arm_shoulder_rotate_controller')
        SetSingleServoSpeed(1.5, 'left_arm_shoulder_rotate_controller')

        # mute the microphone
        self.mic_system_enable_pub.publish(False)
 
        # Move arm into hands up position
        hands_up1()
        time.sleep(1)
        hands_up2()
        time.sleep(3)
        # slow arm to avoid jerk
        SetSingleServoSpeed(0.5, 'right_arm_shoulder_rotate_controller')
        SetSingleServoSpeed(0.5, 'left_arm_shoulder_rotate_controller')
        time.sleep(1)

        # say response
        rospy.loginfo("Talking")
        goal = audio_and_speech_common.msg.speechGoal(text_to_speak="I am not the droid you are looking for")
        client.send_goal(goal)
        result = client.wait_for_result()   # wait for speech to complete
        rospy.loginfo("Speech goal returned result: %d", result)

        time.sleep(1)
        SetSingleServoSpeed(1.5, 'right_arm_shoulder_rotate_controller')
        SetSingleServoSpeed(1.5, 'left_arm_shoulder_rotate_controller')

        # Move head and arms back to ready position
        all_home()
 
        # Finish Behavior
        for i in range(1, 5):
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Behavior preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break

            rospy.loginfo('%s: Running behavior' % (self._action_name))
            self._feedback.running = True
            self._as.publish_feedback(self._feedback)

            r.sleep()
          
        if success:
            rospy.loginfo('%s: Behavior complete' % self._action_name)
            self._as.set_succeeded(self._result)

        time.sleep(1)
        # un-mute the microphone
        self.mic_system_enable_pub.publish(True)
 
        
if __name__ == '__main__':
    rospy.init_node('hands_up_behavior')
    server = BehaviorAction(rospy.get_name())
    rospy.spin()
