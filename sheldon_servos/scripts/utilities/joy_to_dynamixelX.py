#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
import time

from sheldon_servos.standard_servo_positions import *
from sheldon_servos.set_servo_speed import *
from sheldon_servos.set_servo_torque import *


def callback(data):
    if data.buttons[5] == 1: # Right Top Trigger
    	pub_pan.publish(data.axes[2] * -1.0)
    	pub_tilt.publish(data.axes[3] * -1.0)
    	pub_sidetilt.publish(0.0)

        #if data.axes[4] > 0.0:
        #    pub_sidetilt.publish(0.25)
	#elif data.axes[4] < 0.0:
    	#    pub_sidetilt.publish(-0.25)
        #else:
    	#    pub_sidetilt.publish(0.0)

    #if data.buttons[6] == 1:  # Left Bottom Trigger USED BY WHEELS!
    if data.buttons[7] == 1:  # Right Bottom Trigger
    	print("Right Bottom Trigger") # available
 
    # NOTE: Non-Trigger buttons are handled in sheldon_joy_buttons

    
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/joy", Joy, callback)
    rospy.spin()

if __name__ == '__main__':

    listener()
