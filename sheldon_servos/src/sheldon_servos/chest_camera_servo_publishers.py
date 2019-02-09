#!/usr/bin/env python

import roslib
roslib.load_manifest('sheldon_servos')
import rospy
from std_msgs.msg import Float64

# Servo Position Command Publishers
pub_chest_camera_tilt = rospy.Publisher('/chest_camera_tilt_joint/command', Float64, queue_size=1)

