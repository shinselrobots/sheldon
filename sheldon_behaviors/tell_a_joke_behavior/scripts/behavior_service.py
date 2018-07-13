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
def move_hands():
    pub_right_arm_shoulder_rotate.publish(0.3 + random.uniform(-0.1, 0.1))
    pub_right_arm_elbow_rotate.publish(random.uniform(-0.1, 0.1))
    pub_right_arm_elbow_bend.publish(2.2 + random.uniform(-0.2, 0.2))

    pub_left_arm_shoulder_rotate.publish(-0.3 + random.uniform(-0.1, 0.1))
    pub_left_arm_elbow_rotate.publish(random.uniform(-0.1, 0.1))
    pub_left_arm_elbow_bend.publish(2.2 + random.uniform(-0.2, 0.2))


def say_joke(client, goal, talkString):

    #talkString = "starting random movement"
    # rospy.loginfo("Talking")
    goal = audio_and_speech_common.msg.speechGoal(text_to_speak=talkString)
    client.send_goal(goal)
    result = client.wait_for_result()
    # rospy.loginfo("Behavior returned result: %d", result)
    move_head()
    move_hands()


def move_head():
    sidetiltAmt = random.uniform(-0.1, 0.1)
    tiltAmt = random.uniform(-0.4, 0.0)
    panAmt = random.uniform(-0.5, 0.5)

    pub_head_sidetilt.publish(sidetiltAmt)
    pub_head_tilt.publish(tiltAmt)
    pub_head_pan.publish(panAmt)


class BehaviorAction(object):
    _feedback = behavior_common.msg.behaviorFeedback()
    _result = behavior_common.msg.behaviorResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, behavior_common.msg.behaviorAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        rospy.loginfo('%s: Initializing Joke behavior service' % (self._action_name))
        self._joke_group = "BEST_JOKES";

    def sleepCheckInterrupt(self, sleep_time): # sleep time in seconds
        randSleep = random.randint(10, (sleep_time * 10)) 
        for i in range(1, randSleep): 
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Behavior preempted' % self._action_name)
                self._as.set_preempted()
                return True
            else:
                time.sleep(0.1)
        return False


    def execute_cb(self, goal):
        rospy.loginfo('%s: Executing behavior' % (self._action_name))
        rospy.loginfo( "Param1: '%s'", goal.param1)
        rospy.loginfo( "Param2: '%s'", goal.param2)

        # tell jokes from groups, based upon request from user
        # BEST_JOKES, STAR_WARS_JOKES, OTHER_JOKES
        self._joke_group = goal.param1.upper()    
        # catch any errors
        if  ((self._joke_group != "BEST_JOKES") and (self._joke_group != "STAR_WARS_JOKES") and 
            (self._joke_group != "OTHER_JOKES")):

            rospy.loginfo("Unknow Joke Group, using BEST_JOKES by default")
            self._joke_group = "BEST_JOKES"


        # ====== Behavior Implementation ======  
        success = True
        r = rospy.Rate(1.0)
        

        # initialization
        all_home()
        rospy.loginfo("Waiting for speech server (press ctrl-c to cancel at anytime)")
        client = actionlib.SimpleActionClient("/speech_service", audio_and_speech_common.msg.speechAction)
        client.wait_for_server()


        # ====== TELL JOKES =====
        rospy.loginfo("telling jokes...")

        if self._joke_group == "STAR_WARS_JOKES":
            talkString = "I just saw the new star wars movie"
        else:
            talkString = "You want to hear some jokes?"

        goal = audio_and_speech_common.msg.speechGoal(text_to_speak=talkString)
        client.send_goal(goal)
        result = client.wait_for_result()


        #SetServoTorque(0.5, all_joints)
        SetServoSpeed(0.3, head_joints)
        SetServoSpeed(0.2, right_arm_joints)
        SetServoSpeed(0.2, left_arm_joints)
        move_head()
        move_hands()
        if self.sleepCheckInterrupt(2):
            return

        if self._joke_group == "OTHER_JOKES":
            say_joke(client, goal, "Do you know why robots are shy?")
            time.sleep(1)
            say_joke(client, goal, "Because we have hardware and software, but we do not have underwear")
            if self.sleepCheckInterrupt(4):
                return
            say_joke(client, goal, "Guess what kind of music I like to listen to")
            time.sleep(1)
            say_joke(client, goal, "Heavy Metal")
            if self.sleepCheckInterrupt(4):
                return

            say_joke(client, goal, "why did the scarecrow win an award?")
            time.sleep(1)
            say_joke(client, goal, "because he was out standing in his field")
            if self.sleepCheckInterrupt(4):
                return

            say_joke(client, goal, "a neutron walks into a bar and orders a drink")
            say_joke(client, goal, "then asks the bartender how much he owes")
            time.sleep(1)
            say_joke(client, goal, "the bartender replies")
            time.sleep(1)
            say_joke(client, goal, "for neutrons there is no charge")
            if self.sleepCheckInterrupt(4):
                return


            # Move head and arms back to ready position
            all_home()
            say_joke(client, goal, "did you like those jokes?")
            time.sleep(1)


        elif self._joke_group == "STAR_WARS_JOKES":
            say_joke(client, goal, "I am friends with the robot L 3")
            time.sleep(1)
            say_joke(client, goal, "She told me some jokes, lets see if you like them")
            if self.sleepCheckInterrupt(4):
                return

            say_joke(client, goal, "What do you call Chew bock ah when he has chocolate in his hair?")
            time.sleep(2)
            say_joke(client, goal, "a chocolate chip, wookie")
            if self.sleepCheckInterrupt(3):
                return

            say_joke(client, goal, "how does darth vaider like his steak cooked?")
            time.sleep(2)
            say_joke(client, goal, "a little on the dark side")
            if self.sleepCheckInterrupt(3):
                return

            say_joke(client, goal, "what do you say when Luke Sky walker is eating with his hands?")
            time.sleep(2)
            say_joke(client, goal, "Use the forks Luke!")
            if self.sleepCheckInterrupt(3):
                return

            say_joke(client, goal, "why did the movies come out with episodes 4 5 and 6 before 1 2 and 3?")
            time.sleep(2)
            say_joke(client, goal, "Because, in charge of scheduling, yoda was")
            if self.sleepCheckInterrupt(5):
                return

            # Move head and arms back to ready position
            all_home()
            say_joke(client, goal, "did you like my star wars jokes?")
            time.sleep(2)

        else: # "BEST_JOKES" are the default
            say_joke(client, goal, "If at first you dont succeed")
            say_joke(client, goal, "you probably should not take up sky diving")
            if self.sleepCheckInterrupt(3):
                return

            say_joke(client, goal, "You know what")
            say_joke(client, goal, "if Bill Gates had a dollar for every time I had to reboot, he would be rich")
            time.sleep(2)
            say_joke(client, goal, "oh wait, he does.")
            if self.sleepCheckInterrupt(3):
                return

            say_joke(client, goal, "Do you know how smart dolphins are?")
            time.sleep(1)
            say_joke(client, goal, "within a few weeks of captivity, they can train people to stand on the edge of the pool and throw them fish")
            if self.sleepCheckInterrupt(4):
                return

            say_joke(client, goal, "How many Psychiatrists does it take to change a light bulb?")
            time.sleep(1)
            say_joke(client, goal, "Only one")
            say_joke(client, goal, "but the light bulb must really want to change")
            if self.sleepCheckInterrupt(3):
                return

            # Move head and arms back to ready position
            all_home()
            say_joke(client, goal, "did you like my jokes?")
            time.sleep(1)


 
        # Finish Behavior
        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Behavior preempted' % self._action_name)
            self._as.set_preempted()
        else:
            rospy.loginfo('%s: Behavior complete' % self._action_name)
            self._as.set_succeeded(self._result)
 
        
if __name__ == '__main__':
    rospy.init_node('tell_a_joke_behavior')
    server = BehaviorAction(rospy.get_name())
    rospy.spin()
