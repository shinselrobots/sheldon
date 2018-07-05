#!/usr/bin/env python
import rospy
import logging
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_msgs.msg import UInt16
from behavior_common.msg import CommandState

def callback(data):
    msg = CommandState()


    # we only allow one behavior state at a time

    if data.buttons[0] == 1:  # Blue X button - Ship Position
      msg.commandState = "SHIP"
      pub_behavior.publish(msg) 
    elif data.buttons[1] == 1:  # Green A button - Follow Me
      msg.commandState = "FOLLOW_ME"
      pub_behavior.publish(msg) 
    elif data.buttons[2] == 1:    # Red B button - Stop all behaviors
      msg.commandState = "STOP"
      pub_behavior.publish(msg) 
    elif data.buttons[3] == 1:  # Yellow Y button - Wave
      msg.commandState = "WAVE"
      pub_behavior.publish(msg) 
    elif data.buttons[8] == 1:  # "Back" button - Sleep
      msg.commandState = "SLEEP"
      pub_behavior.publish(msg) 
    elif data.buttons[9] == 1:  # "Start" button - Wakeup
      msg.commandState = "WAKEUP"
      pub_behavior.publish(msg) 

    elif data.buttons[7] == 1:  # "Right bottom trigger - use for testing stuff
      pub_mark_time.publish(1)  # just publish a marker

    for i in range(0, 11):
      if data.buttons[i] == 1: 
        rospy.loginfo("Button %d Pressed", i)


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/joy", Joy, callback)
    rospy.loginfo("Joy_buttons Ready")

    rospy.spin()

if __name__ == '__main__':
    # Publish an action for the behavior engine to handle
    pub_behavior = rospy.Publisher('behavior/cmd', CommandState, queue_size=2)
    pub_mark_time = rospy.Publisher('/user_mark_time', UInt16, queue_size=1)
    listener()
