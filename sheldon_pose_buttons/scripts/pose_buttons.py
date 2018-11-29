#!/usr/bin/env python
# Handle buttons on sheldon to allow arms to be positioned
# Implementation note:  
#   This could (should?) be done with arrays instead of individual variables for each servo,
#   but I wanted to be sure I was getting the right servos in each position (for now)


import rospy
import logging
#import time
#import csv
#import math
#import sys

from sensor_msgs.msg import JointState
from std_msgs.msg import UInt16
from std_msgs.msg import Bool


# SHELDON ONLY
#from dynamixel_controllers.srv import TorqueEnable, SetServoTorqueLimit, SetSpeed
from sheldon_servos.servo_joint_list import all_joints, head_joints, right_arm_joints, left_arm_joints
from sheldon_servos.head_servo_publishers import *

from sheldon_servos.standard_servo_positions import *
from sheldon_servos.set_servo_speed import *
from sheldon_servos.set_servo_torque import *

class PoseButtons():
    def __init__(self):
        rospy.init_node('sheldon_pose_buttons')

        self.joint_state = JointState()

        # subscribe to servo position messages
        servo_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_cb)

        # subscribe to arm button messages
        button_right_sub = rospy.Subscriber('/arm_button_right', Bool, self.arm_button_right_cb)
        button_left_sub = rospy.Subscriber('/arm_button_left', Bool, self.arm_button_left_cb)


        # Head Joints
        self.head_pan_joint = rospy.get_param('~head_pan_joint', 'head_pan_joint')
        self.head_pan = 0.0
        self.last_head_pan = 0.0
        self.head_tilt_joint = rospy.get_param('~head_tilt_joint', 'head_tilt_joint')
        self.head_tilt = 0.0
        self.last_head_tilt = 0.0
        self.head_sidetilt_joint = rospy.get_param('~head_sidetilt_joint', 'head_sidetilt_joint')
        self.head_sidetilt = 0.0
        self.last_head_sidetilt = 0.0

        # Right Arm Joints
        self.right_arm_shoulder_rotate_joint = rospy.get_param(
            '~right_arm_shoulder_rotate_joint', 'right_arm_shoulder_rotate_joint')
        self.right_arm_shoulder_rotate = 0.0
        self.last_right_arm_shoulder_rotate = 0.0
        self.right_arm_shoulder_lift_joint = rospy.get_param(
            '~right_arm_shoulder_lift_joint', 'right_arm_shoulder_lift_joint')
        self.right_arm_shoulder_lift = 0.0
        self.last_right_arm_shoulder_lift = 0.0
        self.right_arm_elbow_rotate_joint = rospy.get_param(
            '~right_arm_elbow_rotate_joint', 'right_arm_elbow_rotate_joint')
        self.right_arm_elbow_rotate = 0.0
        self.last_right_arm_elbow_rotate = 0.0
        self.right_arm_elbow_bend_joint = rospy.get_param(
            '~right_arm_elbow_bend_joint', 'right_arm_elbow_bend_joint')
        self.right_arm_elbow_bend = 0.0
        self.last_right_arm_elbow_bend = 0.0
        self.right_arm_wrist_bend_joint = rospy.get_param(
            '~right_arm_wrist_bend_joint', 'right_arm_wrist_bend_joint')
        self.right_arm_wrist_bend = 0.0
        self.last_right_arm_wrist_bend = 0.0
        self.right_arm_wrist_rotate_joint = rospy.get_param(
            '~right_arm_wrist_rotate_joint', 'right_arm_wrist_rotate_joint')
        self.right_arm_wrist_rotate = 0.0
        self.last_right_arm_wrist_rotate = 0.0
        self.right_arm_gripper_finger_joint = rospy.get_param(
            '~right_arm_gripper_finger_joint', 'right_arm_gripper_finger_joint')
        self.right_arm_gripper_finger = 0.0
        self.last_right_arm_gripper_finger = 0.0


        # Left Arm Joints
        self.left_arm_shoulder_rotate_joint = rospy.get_param(
            '~left_arm_shoulder_rotate_joint', 'left_arm_shoulder_rotate_joint')
        self.left_arm_shoulder_rotate = 0.0
        self.last_left_arm_shoulder_rotate = 0.0
        self.left_arm_shoulder_lift_joint = rospy.get_param(
            '~left_arm_shoulder_lift_joint', 'left_arm_shoulder_lift_joint')
        self.left_arm_shoulder_lift = 0.0
        self.last_left_arm_shoulder_lift = 0.0
        self.left_arm_elbow_rotate_joint = rospy.get_param(
            '~left_arm_elbow_rotate_joint', 'left_arm_elbow_rotate_joint')
        self.left_arm_elbow_rotate = 0.0
        self.last_left_arm_elbow_rotate = 0.0
        self.left_arm_elbow_bend_joint = rospy.get_param(
            '~left_arm_elbow_bend_joint', 'left_arm_elbow_bend_joint')
        self.left_arm_elbow_bend = 0.0
        self.last_left_arm_elbow_bend = 0.0
        self.left_arm_wrist_bend_joint = rospy.get_param(
            '~left_arm_wrist_bend_joint', 'left_arm_wrist_bend_joint')
        self.left_arm_wrist_bend = 0.0
        self.last_left_arm_wrist_bend = 0.0
        self.left_arm_wrist_rotate_joint = rospy.get_param(
            '~left_arm_wrist_rotate_joint', 'left_arm_wrist_rotate_joint')
        self.left_arm_wrist_rotate = 0.0
        self.last_left_arm_wrist_rotate = 0.0
        self.left_arm_gripper_finger_joint = rospy.get_param(
            '~left_arm_gripper_finger_joint', 'left_arm_gripper_finger_joint')
        self.left_arm_gripper_finger = 0.0
        self.last_left_arm_gripper_finger = 0.0

        rospy.loginfo("pose_buttons ready")

    # BUTTON EVENTS.  
    # Button Down used to relax servos for positioning, Button Up to log servo positions
    def arm_button_right_cb(self, msg):
        button_pressed = msg.data

        #rospy.loginfo("DEBUG got right arm button event.")
        if button_pressed:
            rospy.loginfo("right arm button down event. Removing arm torque.")
            SetServoTorque(0.0, right_arm_joints) # remove torque

        else:
            rospy.loginfo("right arm button up event. Logging servo positions")
            self.marker_flag_set = True    # Force current servo positions to be written
            SetServoTorque(0.5, right_arm_joints) # restore torque

            # PRINT SERVO VALUES
            self.print_arm_servos_right()

    def arm_button_left_cb(self, msg):
        button_pressed = msg.data

        #rospy.loginfo("DEBUG got left arm button event.")
        if button_pressed:
            rospy.loginfo("left arm button down event. Removing arm torque.")
            SetServoTorque(0.0, left_arm_joints) # remove torque

        else:
            rospy.loginfo("left arm button up event. Logging servo positions")
            self.marker_flag_set = True    # Force current servo positions to be written
            SetServoTorque(0.5, left_arm_joints) # restore torque

            # PRINT SERVO VALUES
            self.print_arm_servos_left()

    def joint_state_cb(self, msg):
        #rospy.loginfo("joint_state_cb called")

        try:
            test = msg.name.index(self.head_pan_joint)
            self.joint_state = msg
            #rospy.loginfo("POSE BUTTONS: Got servo message!") 
        except:
            #rospy.loginfo("POSE BUTTONS: Not a servo message, skipping...") 
            return

       # Get the servo positions
        try:
            self.head_pan = self.joint_state.position[
              self.joint_state.name.index(self.head_pan_joint)]
            self.head_tilt = self.joint_state.position[
              self.joint_state.name.index(self.head_tilt_joint)]
            self.head_sidetilt = self.joint_state.position[
              self.joint_state.name.index(self.head_sidetilt_joint)]

            self.right_arm_shoulder_rotate = self.joint_state.position[
              self.joint_state.name.index(self.right_arm_shoulder_rotate_joint)]
            self.right_arm_shoulder_lift = self.joint_state.position[
              self.joint_state.name.index(self.right_arm_shoulder_lift_joint)]
            self.right_arm_elbow_rotate = self.joint_state.position[
              self.joint_state.name.index(self.right_arm_elbow_rotate_joint)]
            self.right_arm_elbow_bend = self.joint_state.position[
              self.joint_state.name.index(self.right_arm_elbow_bend_joint)]
            self.right_arm_wrist_bend = self.joint_state.position[
              self.joint_state.name.index(self.right_arm_wrist_bend_joint)]
            self.right_arm_wrist_rotate = self.joint_state.position[
              self.joint_state.name.index(self.right_arm_wrist_rotate_joint)]
            self.right_arm_gripper_finger = self.joint_state.position[
              self.joint_state.name.index(self.right_arm_gripper_finger_joint)]


            self.left_arm_shoulder_rotate = self.joint_state.position[
              self.joint_state.name.index(self.left_arm_shoulder_rotate_joint)]
            self.left_arm_shoulder_lift = self.joint_state.position[
              self.joint_state.name.index(self.left_arm_shoulder_lift_joint)]
            self.left_arm_elbow_rotate = self.joint_state.position[
              self.joint_state.name.index(self.left_arm_elbow_rotate_joint)]
            self.left_arm_elbow_bend = self.joint_state.position[
              self.joint_state.name.index(self.left_arm_elbow_bend_joint)]
            self.left_arm_wrist_bend = self.joint_state.position[
              self.joint_state.name.index(self.left_arm_wrist_bend_joint)]
            self.left_arm_wrist_rotate = self.joint_state.position[
              self.joint_state.name.index(self.left_arm_wrist_rotate_joint)]
            self.left_arm_gripper_finger = self.joint_state.position[
              self.joint_state.name.index(self.left_arm_gripper_finger_joint)]

        except:
            # rospy.logerr("POSE BUTTONS: ERROR getting servo values") 
            return

        debug_servo_values = False
        if debug_servo_values:
            rospy.loginfo("HEAD: Pan = %1.4f, Tilt = %1.4f, Sidetilt = %1.4f", 
                self.head_pan, self.head_tilt, self.head_sidetilt)
            rospy.loginfo("RIGHT ARM: ShoulderRotate = %f, ShoulderLift = %f, ElbowRotate = %f, ElbowBend = %f, WristRotate = %f, Gripper = %f", 
                self.right_arm_shoulder_rotate, self.right_arm_shoulder_lift, 
                self.right_arm_elbow_rotate, self.right_arm_elbow_bend, 
                self.right_arm_elbow_rotate, self.right_arm_wrist_bend, 
                self.right_arm_wrist_rotate, self.right_arm_gripper_finger)
            rospy.loginfo("LEFT ARM: ShoulderRotate = %f, ShoulderLift = %f, ElbowRotate = %f, ElbowBend = %f, WristRotate = %f, Gripper = %f", 
                self.left_arm_shoulder_rotate, self.left_arm_shoulder_lift, 
                self.left_arm_elbow_rotate, self.left_arm_elbow_bend, 
                self.left_arm_elbow_rotate, self.left_arm_wrist_bend, 
                self.left_arm_wrist_rotate, self.left_arm_gripper_finger)



    def print_arm_servos_right(self):

        rospy.loginfo("-----------------------------------")
        rospy.loginfo("right_arm_shoulder_rotate = %7.4f", self.right_arm_shoulder_rotate)
        rospy.loginfo("right_arm_shoulder_lift   = %7.4f", self.right_arm_shoulder_lift)
        rospy.loginfo("right_arm_elbow_rotate    = %7.4f", self.right_arm_elbow_rotate)
        rospy.loginfo("right_arm_elbow_bend      = %7.4f", self.right_arm_elbow_bend)
        rospy.loginfo("right_arm_wrist_bend      = %7.4f", self.right_arm_wrist_bend)
        rospy.loginfo("right_arm_wrist_rotate    = %7.4f", self.right_arm_wrist_rotate)
        rospy.loginfo("right_arm_gripper_finger  = %7.4f", self.right_arm_gripper_finger)
        rospy.loginfo("-----------------------------------")

    def print_arm_servos_left(self):
        rospy.loginfo("-----------------------------------")
        rospy.loginfo("left_arm_shoulder_rotate = %7.4f", self.left_arm_shoulder_rotate)
        rospy.loginfo("left_arm_shoulder_lift   = %7.4f", self.left_arm_shoulder_lift)
        rospy.loginfo("left_arm_elbow_rotate    = %7.4f", self.left_arm_elbow_rotate)
        rospy.loginfo("left_arm_elbow_bend      = %7.4f", self.left_arm_elbow_bend)
        rospy.loginfo("left_arm_wrist_bend      = %7.4f", self.left_arm_wrist_bend)
        rospy.loginfo("left_arm_wrist_rotate    = %7.4f", self.left_arm_wrist_rotate)
        rospy.loginfo("left_arm_gripper_finger  = %7.4f", self.left_arm_gripper_finger)
        rospy.loginfo("-----------------------------------")


        
if __name__=='__main__':

        PoseButtons()
        rospy.spin()



