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

# Globals
def touch_ground1():
    pub_right_arm_shoulder_rotate.publish(-1.55)
    pub_left_arm_shoulder_rotate.publish(1.55)

def touch_ground2():
    head_home()
    pub_right_arm_shoulder_lift.publish(0.0)
    pub_right_arm_elbow_rotate.publish(0.0)
    pub_right_arm_wrist_rotate.publish(0.0)
    pub_right_arm_gripper_finger.publish(-0.5)

    pub_left_arm_shoulder_lift.publish(0.0)
    pub_left_arm_elbow_rotate.publish(0.0)
    pub_left_arm_wrist_rotate.publish(0.0)
    pub_left_arm_gripper_finger.publish(-0.5)

    time.sleep(2.0)
    pub_right_arm_elbow_bend.publish(0.0)
    pub_left_arm_elbow_bend.publish(0.0)



class BehaviorAction(object):
    _feedback = behavior_common.msg.behaviorFeedback()
    _result = behavior_common.msg.behaviorResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, behavior_common.msg.behaviorAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        rospy.loginfo('%s: Initializing Wave behavior service' % (self._action_name))

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

        SetServoTorque(0.3, all_servo_joints)
        SetServoSpeed(0.5, all_servo_joints)
        SetSingleServoSpeed(1.8, 'right_arm_shoulder_rotate_joint')
        SetSingleServoSpeed(1.8, 'left_arm_shoulder_rotate_joint')
 
        goal = audio_and_speech_common.msg.speechGoal(text_to_speak="Moving into Shipping Mode")
        client.send_goal(goal)
        result = client.wait_for_result()   # wait for speech to complete
        rospy.loginfo("Speech goal returned result: %d", result)
        time.sleep(0.4)

        # start moving into various positions
        #wave2()
        #time.sleep(0.4)
        # Move arm into start wave position
        #wave1()
        #time.sleep(2)

        #waist_bow_down()
        touch_ground1()
        time.sleep(2)
        #waist_full_down()

        touch_ground2()
        time.sleep(4)

        # Move back to ready position
        SetServoTorque(0.5, all_servo_joints)
        waist_home()
        time.sleep(1)
        all_home()


        # Finish Behavior
        # while True:
        for i in range(1, 5):
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Behavior preempted' % self._action_name)
                SetServoTorque(0.5, right_arm_joints)
                all_home()
                waist_home()


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
 
        
if __name__ == '__main__':
    rospy.init_node('joint_test_behavior')
    server = BehaviorAction(rospy.get_name())
    rospy.spin()
