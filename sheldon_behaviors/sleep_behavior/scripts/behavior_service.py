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
        rospy.loginfo('%s: Initializing Sleep behavior service' % (self._action_name))
      
    def execute_cb(self, goal):
        rospy.loginfo('%s: DAVE>>> Executing behavior' % (self._action_name))

        rospy.loginfo( "Param1: '%s'", goal.param1)
        rospy.loginfo( "Param2: '%s'", goal.param2)

        # =========== Behavior Implementation ==============  
        success = True
        r = rospy.Rate(1.0)

        pub_eye_cmd = rospy.Publisher('/head/eye_cmd', UInt16, queue_size=10)        
        pub_light_mode = rospy.Publisher('/arm_led_mode', UInt16, queue_size=10)        
        pub_ear_cmd = rospy.Publisher('/head/ear_cmd', UInt16, queue_size=10)        

        rospy.loginfo("Waiting for speech server (press ctrl-c to cancel at anytime)")
        client = actionlib.SimpleActionClient("/speech_service", audio_and_speech_common.msg.speechAction)
        client.wait_for_server()

        rospy.loginfo("Talking...")
        goal = audio_and_speech_common.msg.speechGoal(text_to_speak="shutting down")
        client.send_goal(goal)
        result = client.wait_for_result() # wait for speech to complete
        rospy.loginfo("Speech goal returned result: %d", result)

        rospy.loginfo("DAVE DDAVE DAVE DABVE DAVE ")

        # Move head and arms to sleep position
        SetServoTorque(0.8, all_joints)
        SetServoSpeed(0.5, head_joints)
        SetServoSpeed(1.0, right_arm_joints)
        SetServoSpeed(1.0, left_arm_joints)

        # Move elbows at fast speed to lock
        SetSingleServoSpeed(2.0, "right_arm_elbow_bend_joint")
        SetSingleServoSpeed(2.0, "left_arm_elbow_bend_joint")
        time.sleep(0.5)


        all_sleep()    # Move all servos to sleep position 1
        time.sleep(2)

        # lock arms
        pub_right_arm_elbow_bend.publish(3.00)
        pub_left_arm_elbow_bend.publish(3.13)

        time.sleep(1)



        # Turn off servo torque
        rospy.loginfo("Turning off servo torque and eyes")
        SetServoTorque(0.0, all_joints)
        pub_eye_cmd.publish(0) # 0 = Turn eyes off
        pub_ear_cmd.publish(0) # 0 = Turn ear lights off
        pub_light_mode.publish(0) # 0 = Turn lights off


        time.sleep(5.0) # seconds
        rospy.loginfo('  Sleep Complete.  Running until some other behavior preempts, to suppress Idle behavior...')

        #rospy.loginfo('%s: Running behavior' % (self._action_name))
        self._feedback.running = True
        self._as.publish_feedback(self._feedback)

        # Run forever to keep Idle behavior from running.
        # may be prempted by any other behavior (such as wake)
        while True:
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Behavior preempted' % self._action_name)
                self._as.set_preempted()
                success = True
                break

            r.sleep()
          
        if success:
            rospy.loginfo('%s: Behavior complete' % self._action_name)
            self._as.set_succeeded(self._result)

        
if __name__ == '__main__':
    rospy.init_node('sleep_behavior')
    server = BehaviorAction(rospy.get_name())
    rospy.spin()
