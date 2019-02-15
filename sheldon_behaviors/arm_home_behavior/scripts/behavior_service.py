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
        rospy.loginfo('%s: Initializing behavior service' % (self._action_name))

    def execute_cb(self, goal):
        rospy.loginfo('%s: Executing behavior' % (self._action_name))
        rospy.loginfo( "Param1: '%s'", goal.param1) # Right, Left, or Both
        rospy.loginfo( "Param2: '%s'", goal.param2)

        self._feedback.running = True
        self._as.publish_feedback(self._feedback)

        # ====== Behavior Implementation ======  
        SetServoTorque(0.5, all_servo_joints)
        SetServoSpeed(0.5, all_servo_joints)
 
        # Move arm(s) into home position
        if goal.param1.to_lower() == 'all':
            left_arm_home()
            right_arm_home()
        elif goal.param1.to_lower() == 'right':
            right_arm_home()
        if goal.param1.to_lower() == 'left':
            left_arm_home()

        # happens so fast, not really preemptable
        rospy.loginfo('%s: Behavior complete' % self._action_name)
        self._as.set_succeeded(self._result)
 
        
if __name__ == '__main__':
    rospy.init_node('arm_home_behavior')
    server = BehaviorAction(rospy.get_name())
    rospy.spin()
