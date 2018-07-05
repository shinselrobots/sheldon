#!/usr/bin/env python

import roslib
roslib.load_manifest('sheldon_servos')
import rospy
from std_msgs.msg import Float64

# Servo Position Command Publishers
pub_right_arm_shoulder_rotate = rospy.Publisher('/right_arm_shoulder_rotate_controller/command', Float64, queue_size=1)
pub_right_arm_shoulder_lift = rospy.Publisher('/right_arm_shoulder_lift_controller/command', Float64, queue_size=1)
pub_right_arm_elbow_rotate = rospy.Publisher('/right_arm_elbow_rotate_controller/command', Float64, queue_size=1)
pub_right_arm_elbow_bend = rospy.Publisher('/right_arm_elbow_bend_controller/command', Float64, queue_size=1)
pub_right_arm_wrist_rotate = rospy.Publisher('/right_arm_wrist_rotate_controller/command', Float64, queue_size=1)
pub_right_arm_claw = rospy.Publisher('/right_arm_claw_controller/command', Float64, queue_size=1)

