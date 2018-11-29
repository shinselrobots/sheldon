#!/usr/bin/env python

import roslib
roslib.load_manifest('sheldon_servos')
import rospy
from std_msgs.msg import Float64

# Servo Position Command Publishers

pub_left_arm_shoulder_rotate = rospy.Publisher('/left_arm_shoulder_rotate_joint/command', Float64, queue_size=1)
pub_left_arm_shoulder_lift = rospy.Publisher('/left_arm_shoulder_lift_joint/command', Float64, queue_size=1)
pub_left_arm_elbow_rotate = rospy.Publisher('/left_arm_elbow_rotate_joint/command', Float64, queue_size=1)
pub_left_arm_elbow_bend = rospy.Publisher('/left_arm_elbow_bend_joint/command', Float64, queue_size=1)
pub_left_arm_wrist_bend = rospy.Publisher('/left_arm_wrist_bend_joint/command', Float64, queue_size=1)
pub_left_arm_wrist_rotate = rospy.Publisher('/left_arm_wrist_rotate_joint/command', Float64, queue_size=1)
pub_left_arm_gripper_finger = rospy.Publisher('/left_arm_gripper_finger_joint/command', Float64, queue_size=1)

