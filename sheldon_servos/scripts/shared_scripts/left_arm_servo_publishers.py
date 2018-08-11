#!/usr/bin/env python

import roslib
roslib.load_manifest('sheldon_servos')
import rospy
from std_msgs.msg import Float64

# Servo Position Command Publishers

pub_left_arm_shoulder_rotate = rospy.Publisher('/left_arm_shoulder_rotate_controller/command', Float64, queue_size=1)
pub_left_arm_shoulder_lift = rospy.Publisher('/left_arm_shoulder_lift_controller/command', Float64, queue_size=1)
pub_left_arm_elbow_rotate = rospy.Publisher('/left_arm_elbow_rotate_controller/command', Float64, queue_size=1)
pub_left_arm_elbow_bend = rospy.Publisher('/left_arm_elbow_bend_controller/command', Float64, queue_size=1)
pub_left_arm_wrist_rotate = rospy.Publisher('/left_arm_wrist_rotate_controller/command', Float64, queue_size=1)
pub_left_arm_gripper = rospy.Publisher('/left_arm_gripper_controller/command', Float64, queue_size=1)

