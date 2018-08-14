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
def wave1():
    pub_right_arm_shoulder_rotate.publish(1.2)
    pub_right_arm_shoulder_lift.publish(0.0)
    pub_right_arm_elbow_rotate.publish(0.0)
    pub_right_arm_elbow_bend.publish(2.0)
    pub_right_arm_wrist_rotate.publish(0.0)
    pub_right_arm_gripper.publish(0.0)

    head_home() # look slightly up at people
    #pub_head_sidetilt.publish(0.0)
    #pub_head_tilt.publish(0.0)
    #pub_head_pan.publish(0.0)

def wave2():
    print("-----> wave position 2")
    pub_right_arm_elbow_rotate.publish(-0.2)

def wave3():
    print("-----> wave position 3")
    pub_right_arm_elbow_rotate.publish(0.2)

def wave4():
    print("-----> wave position 4")
    pub_right_arm_elbow_rotate.publish(0.0)


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

        self.foo = True      

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

        SetServoTorque(0.5, right_arm_joints)
        SetServoSpeed(0.7, right_arm_joints)
        SetSingleServoSpeed(1.8, 'right_arm_shoulder_rotate_joint')


        self.foo = False
 
        # mute the microphone
        self.mic_system_enable_pub.publish(False)

        # Move arm into start wave position
        wave1()
        time.sleep(2)

        # say Hi
        rospy.loginfo("Saying Hello")
        goal = audio_and_speech_common.msg.speechGoal(text_to_speak="hello")
        client.send_goal(goal)
        #result = client.wait_for_result()   # DONT wait for speech to complete
        #rospy.loginfo("Speech goal returned result: %d", result)

        # start waving while talking
        wave2()
        time.sleep(0.4)
        wave3()
        time.sleep(0.4)
        wave2()
        time.sleep(0.4)
        wave4()
        time.sleep(0.4)

        # Move head and arms back to ready position
        all_home()
        #pub_right_arm_gripper.publish(-2.0)

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

        # un-mute the microphone
        self.mic_system_enable_pub.publish(True)
 
        
if __name__ == '__main__':
    rospy.init_node('wave_behavior')
    server = BehaviorAction(rospy.get_name())
    rospy.spin()
