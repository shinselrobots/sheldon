#!/usr/bin/env python

import roslib
roslib.load_manifest('sheldon_servos')
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Empty

# Servo Position Command Publishers
pub_waist_calibrate = rospy.Publisher('/waist_calibrate', Empty, queue_size=1)
pub_waist_position = rospy.Publisher('/waist_goal_position', Float32, queue_size=1)

