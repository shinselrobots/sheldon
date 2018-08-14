#!/usr/bin/env python

import roslib
roslib.load_manifest('sheldon_servos')
import rospy, time
from std_msgs.msg import Float64

# Servo Position Command Publishers
# ================================================================
pub_head_pan = rospy.Publisher('/head_pan_joint/command', Float64, queue_size=1)
pub_head_tilt = rospy.Publisher('/head_tilt_joint/command', Float64, queue_size=1)
pub_head_sidetilt = rospy.Publisher('/head_sidetilt_joint/command', Float64, queue_size=1)

pub_left_arm_shoulder_rotate = rospy.Publisher('/left_arm_shoulder_rotate_joint/command', Float64, queue_size=1)
pub_left_arm_shoulder_lift = rospy.Publisher('/left_arm_shoulder_lift_joint/command', Float64, queue_size=1)
pub_left_arm_elbow_rotate = rospy.Publisher('/left_arm_elbow_rotate_joint/command', Float64, queue_size=1)
pub_left_arm_elbow_bend = rospy.Publisher('/left_arm_elbow_bend_joint/command', Float64, queue_size=1)
pub_left_arm_wrist_rotate = rospy.Publisher('/left_arm_wrist_rotate_joint/command', Float64, queue_size=1)
pub_left_arm_gripper = rospy.Publisher('/left_arm_gripper_controller/command', Float64, queue_size=1)

pub_right_arm_shoulder_rotate = rospy.Publisher('/right_arm_shoulder_rotate_joint/command', Float64, queue_size=1)
pub_right_arm_shoulder_lift = rospy.Publisher('/right_arm_shoulder_lift_joint/command', Float64, queue_size=1)
pub_right_arm_elbow_rotate = rospy.Publisher('/right_arm_elbow_rotate_joint/command', Float64, queue_size=1)
pub_right_arm_elbow_bend = rospy.Publisher('/right_arm_elbow_bend_joint/command', Float64, queue_size=1)
pub_right_arm_wrist_rotate = rospy.Publisher('/right_arm_wrist_rotate_joint/command', Float64, queue_size=1)
pub_right_arm_gripper = rospy.Publisher('/right_arm_gripper_controller/command', Float64, queue_size=1)

# ================================================================


