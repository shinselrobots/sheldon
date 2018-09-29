#!/usr/bin/env python

import roslib
roslib.load_manifest('sheldon_servos')
import rospy
from std_msgs.msg import Float64

# Servo Position Command Publishers
pub_head_pan = rospy.Publisher('/head_pan_joint/command', Float64, queue_size=1)
pub_head_tilt = rospy.Publisher('/head_tilt_joint/command', Float64, queue_size=1)
pub_head_sidetilt = rospy.Publisher('/head_sidetilt_joint/command', Float64, queue_size=1)

