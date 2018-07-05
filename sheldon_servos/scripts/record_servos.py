#!/usr/bin/env python
# Record servo positions for later scripting of robot behavior
# Implementation note:  
#   This could (should?) be done with arrays instead of individual variables for each servo,
#   but I wanted to be sure I was getting the right servos in each position (for now)
# If a parser is used, it should ignore any lines that start with '#'


import rospy
import logging
import time
import csv
import math
import sys

from sensor_msgs.msg import JointState
from std_msgs.msg import UInt16


# SHELDON ONLY
#from dynamixel_controllers.srv import TorqueEnable, SetServoTorqueLimit, SetSpeed
from sheldon_servos.servo_joint_list import all_joints, head_joints, right_arm_joints
from sheldon_servos.head_servo_publishers import *

from sheldon_servos.standard_servo_positions import *
from sheldon_servos.set_servo_speed import *
from sheldon_servos.set_servo_torque import *

# Tune this to make movements not jerkey by too many intermediate positions
# Head probably likes smaller numbers, arms bigger numbers...
MOVEMENT_THRESHOLD = 0.087 # 5 degrees
#MOVEMENT_THRESHOLD = 0.175 # 10 degrees
#MOVEMENT_THRESHOLD = 0.349 # 20 degrees

FIXED_STEP_SECONDS = 0.960

class RecordServoPositions():
    def __init__(self, option):
        rospy.init_node('record_servos')

        self.record_option = option
        self.joint_state = JointState()
        self.marker_flag_set = False
        self.step_number = 0

        # subscribe to servo position messages
        servo_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_cb)

        # subscribe to marker messages by user, to indicate events in the time stream (such as "servos in position #4")
        time_mark_sub = rospy.Subscriber('/user_mark_time', UInt16, self.time_mark_cb) # arbitrary ID assigned by user

        # Use Python's CSV writer
        self.csv_file = open("/home/system/record_servos.csv", "w") # use "a" to append
        fieldnames = ['comment', 'step', 'time', 'type', 'head_pan', 'head_tilt', 'head_sidetilt', 
            'r_arm', 'right_arm_shoulder_rotate', 'right_arm_shoulder_lift', 
            'right_arm_elbow_rotate', 'right_arm_elbow_bend', 
            'right_arm_wrist_rotate', 'right_arm_claw',
            'l_arm', 'left_arm_shoulder_rotate', 'left_arm_shoulder_lift', 
            'left_arm_elbow_rotate', 'left_arm_elbow_bend', 
            'left_arm_wrist_rotate', 'left_arm_claw', 'param1', 'param2']

        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=fieldnames)
        self.csv_writer.writeheader()
        self.start_time = rospy.get_rostime()
        self.last_elapsed_sec = 0.0


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
        self.right_arm_wrist_rotate_joint = rospy.get_param(
            '~right_arm_wrist_rotate_joint', 'right_arm_wrist_rotate_joint')
        self.right_arm_wrist_rotate = 0.0
        self.last_right_arm_wrist_rotate = 0.0
        self.right_arm_claw_joint = rospy.get_param(
            '~right_arm_claw_joint', 'right_arm_claw_joint')
        self.right_arm_claw = 0.0
        self.last_right_arm_claw = 0.0


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
        self.left_arm_wrist_rotate_joint = rospy.get_param(
            '~left_arm_wrist_rotate_joint', 'left_arm_wrist_rotate_joint')
        self.left_arm_wrist_rotate = 0.0
        self.last_left_arm_wrist_rotate = 0.0
        self.left_arm_claw_joint = rospy.get_param(
            '~left_arm_claw_joint', 'left_arm_claw_joint')
        self.left_arm_claw = 0.0
        self.last_left_arm_claw = 0.0

        rospy.loginfo("record_servos Ready")

    def __del__(self):
        self.csv_file.close()

    def time_mark_cb(self, msg):
        rospy.loginfo("got time ID marker from user")
        self.marker_flag_set = True    # Force current servo positions to be written

        # Only record markers if we are not in Marker only mode (where every line is proceeded by a marker)
        if not ("marker" in self.record_option):
            id = msg.data
            rospy.loginfo("================ GOT MARKER! ================") 
            self.csv_writer.writerow({
                'comment': ' ',
                'step': '0 ',
                'time': '0.0  ',
                'type': 'marker',
                'param1': '{:d}'.format(id)})


    def threshold(self, current, prior):
        if self.marker_flag_set:
            return True # force all servos to report current position
        elif ("marker" not in self.record_option) and (math.fabs(current - prior) > MOVEMENT_THRESHOLD):
            rospy.loginfo("================ GOT MOTION! ================") 
            return True
        else:
            return False # float('nan')

    def joint_state_cb(self, msg):


        # rospy.loginfo("joint_state_cb called")

        try:
            test = msg.name.index(self.head_pan_joint)
            self.joint_state = msg
        except:
            # rospy.loginfo("SERVO_RECORDER: Not a servo message, skipping...") 
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
            self.right_arm_wrist_rotate = self.joint_state.position[
              self.joint_state.name.index(self.right_arm_wrist_rotate_joint)]
            self.right_arm_claw = self.joint_state.position[
              self.joint_state.name.index(self.right_arm_claw_joint)]


            self.left_arm_shoulder_rotate = self.joint_state.position[
              self.joint_state.name.index(self.left_arm_shoulder_rotate_joint)]
            self.left_arm_shoulder_lift = self.joint_state.position[
              self.joint_state.name.index(self.left_arm_shoulder_lift_joint)]
            self.left_arm_elbow_rotate = self.joint_state.position[
              self.joint_state.name.index(self.left_arm_elbow_rotate_joint)]
            self.left_arm_elbow_bend = self.joint_state.position[
              self.joint_state.name.index(self.left_arm_elbow_bend_joint)]
            self.left_arm_wrist_rotate = self.joint_state.position[
              self.joint_state.name.index(self.left_arm_wrist_rotate_joint)]
            self.left_arm_claw = self.joint_state.position[
              self.joint_state.name.index(self.left_arm_claw_joint)]


        except:
            rospy.logerr("SERVO_RECORDER: ERROR getting servo values") 
            return

        debug_servo_values = False
        if debug_servo_values:
            rospy.loginfo("HEAD: Pan = %1.4f, Tilt = %1.4f, Sidetilt = %1.4f", 
                self.head_pan, self.head_tilt, self.head_sidetilt)
            rospy.loginfo("RIGHT ARM: ShoulderRotate = %f, ShoulderLift = %f, ElbowRotate = %f, ElbowBend = %f, WristRotate = %f, Claw = %f", 
                self.right_arm_shoulder_rotate, self.right_arm_shoulder_lift, 
                self.right_arm_elbow_rotate, self.right_arm_elbow_bend, 
                self.right_arm_wrist_rotate, self.right_arm_claw)
            rospy.loginfo("LEFT ARM: ShoulderRotate = %f, ShoulderLift = %f, ElbowRotate = %f, ElbowBend = %f, WristRotate = %f, Claw = %f", 
                self.left_arm_shoulder_rotate, self.left_arm_shoulder_lift, 
                self.left_arm_elbow_rotate, self.left_arm_elbow_bend, 
                self.left_arm_wrist_rotate, self.left_arm_claw)


        # Head
        head_pan = float('nan')
        head_tilt = float('nan')
        head_sidetilt = float('nan')
        if ("head" in self.record_option) or ("all" in self.record_option) or (("marker" in self.record_option) and self.marker_flag_set):
            if self.threshold(self.head_pan, self.last_head_pan):
                head_pan = self.head_pan
                self.last_head_pan = self.head_pan
                #rospy.loginfo("DBG: self.head_pan = %f, self.last_head_pan = %f, new_head_pan = %f", self.head_pan, self.last_head_pan, head_pan)
            if self.threshold(self.head_tilt, self.last_head_tilt):
                head_tilt = self.head_tilt
                self.last_head_tilt = self.head_tilt
            if self.threshold(self.head_sidetilt, self.last_head_sidetilt):
                head_sidetilt = self.head_sidetilt
                self.last_head_sidetilt = self.head_sidetilt

        head_movement = ( not math.isnan(head_pan) or not math.isnan(head_tilt) or not math.isnan(head_sidetilt) )

        # Right Arm
        right_arm_shoulder_rotate = float('nan')
        right_arm_shoulder_lift = float('nan')
        right_arm_elbow_rotate = float('nan')
        right_arm_elbow_bend = float('nan')
        right_arm_wrist_rotate = float('nan')
        right_arm_claw = float('nan')
        if ("right" in self.record_option) or ("all" in self.record_option) or (("marker" in self.record_option) and self.marker_flag_set):
            if self.threshold(self.right_arm_shoulder_rotate, self.last_right_arm_shoulder_rotate):
                    right_arm_shoulder_rotate = self.right_arm_shoulder_rotate
                    self.last_right_arm_shoulder_rotate = self.right_arm_shoulder_rotate
            if self.threshold(self.right_arm_shoulder_lift, self.last_right_arm_shoulder_lift):
                    right_arm_shoulder_lift = self.right_arm_shoulder_lift
                    self.last_right_arm_shoulder_lift = self.right_arm_shoulder_lift
            if self.threshold(self.right_arm_elbow_rotate, self.last_right_arm_elbow_rotate):
                    right_arm_elbow_rotate = self.right_arm_elbow_rotate
                    self.last_right_arm_elbow_rotate = self.right_arm_elbow_rotate
            if self.threshold(self.right_arm_elbow_bend, self.last_right_arm_elbow_bend):
                    right_arm_elbow_bend = self.right_arm_elbow_bend
                    self.last_right_arm_elbow_bend = self.right_arm_elbow_bend
            if self.threshold(self.right_arm_wrist_rotate, self.last_right_arm_wrist_rotate):
                    right_arm_wrist_rotate = self.right_arm_wrist_rotate
                    self.last_right_arm_wrist_rotate = self.right_arm_wrist_rotate
            if self.threshold(self.right_arm_claw, self.last_right_arm_claw):
                    right_arm_claw = self.right_arm_claw
                    self.last_right_arm_claw = self.right_arm_claw

        right_arm_movement = ( 
            not math.isnan(right_arm_shoulder_rotate) or not math.isnan(right_arm_shoulder_lift) or 
            not math.isnan(right_arm_elbow_rotate) or not math.isnan(right_arm_elbow_bend) or
            not math.isnan(right_arm_wrist_rotate) or not math.isnan(right_arm_claw) )


        # Left Arm
        left_arm_shoulder_rotate = float('nan')
        left_arm_shoulder_lift = float('nan')
        left_arm_elbow_rotate = float('nan')
        left_arm_elbow_bend = float('nan')
        left_arm_wrist_rotate = float('nan')
        left_arm_claw = float('nan')
        if ("left" in self.record_option) or ("all" in self.record_option) or (("marker" in self.record_option) and self.marker_flag_set):
            if self.threshold(self.left_arm_shoulder_rotate, self.last_left_arm_shoulder_rotate):
                    left_arm_shoulder_rotate = self.left_arm_shoulder_rotate
                    self.last_left_arm_shoulder_rotate = self.left_arm_shoulder_rotate
            if self.threshold(self.left_arm_shoulder_lift, self.last_left_arm_shoulder_lift):
                    left_arm_shoulder_lift = self.left_arm_shoulder_lift
                    self.last_left_arm_shoulder_lift = self.left_arm_shoulder_lift
            if self.threshold(self.left_arm_elbow_rotate, self.last_left_arm_elbow_rotate):
                    left_arm_elbow_rotate = self.left_arm_elbow_rotate
                    self.last_left_arm_elbow_rotate = self.left_arm_elbow_rotate
            if self.threshold(self.left_arm_elbow_bend, self.last_left_arm_elbow_bend):
                    left_arm_elbow_bend = self.left_arm_elbow_bend
                    self.last_left_arm_elbow_bend = self.left_arm_elbow_bend
            if self.threshold(self.left_arm_wrist_rotate, self.last_left_arm_wrist_rotate):
                    left_arm_wrist_rotate = self.left_arm_wrist_rotate
                    self.last_left_arm_wrist_rotate = self.left_arm_wrist_rotate
            if self.threshold(self.left_arm_claw, self.last_left_arm_claw):
                    left_arm_claw = self.left_arm_claw
                    self.last_left_arm_claw = self.left_arm_claw

        left_arm_movement = ( 
            not math.isnan(left_arm_shoulder_rotate) or not math.isnan(left_arm_shoulder_lift) or
            not math.isnan(left_arm_elbow_rotate) or not math.isnan(left_arm_elbow_bend) or
            not math.isnan(left_arm_wrist_rotate) or not math.isnan(left_arm_claw) )


        # row types:  move, speed (to set servo speeds), speak, lights, etc.
        row_type = 'move'

        if head_movement or right_arm_movement or left_arm_movement or self.marker_flag_set:
            rospy.loginfo("DBG: ========================= WRITING! ======================")
            self.marker_flag_set = False

            elapsed_time = rospy.Time.now() - self.start_time
            elapsed_sec = elapsed_time.to_sec()
            step_seconds = elapsed_sec - self.last_elapsed_sec # log the time between steps
            rospy.loginfo("DBG: step_seconds = %f, elapsed_sec = %f, last_elapsed_sec = %f", 
                step_seconds, elapsed_sec, self.last_elapsed_sec)
            self.last_elapsed_sec = elapsed_sec

            if "marker" in self.record_option: 
                step_seconds = FIXED_STEP_SECONDS

            self.step_number += 1

            self.csv_writer.writerow({
                'comment': ' ',
                'step': '{:d}'.format(self.step_number),
                'time': '{:1.3f}'.format(step_seconds),
                'type': row_type,
                'head_pan': '{:1.4f}'.format(head_pan), 
                'head_tilt': '{:1.4f}'.format(head_tilt),
                'head_sidetilt': '{:1.4f}'.format(head_sidetilt),

                'r_arm': 'R:',      # make file easier for humans to read
                'right_arm_shoulder_rotate': '{:1.4f}'.format(right_arm_shoulder_rotate), 
                'right_arm_shoulder_lift': '{:1.4f}'.format(right_arm_shoulder_lift), 
                'right_arm_elbow_rotate': '{:1.4f}'.format(right_arm_elbow_rotate), 
                'right_arm_elbow_bend': '{:1.4f}'.format(right_arm_elbow_bend), 
                'right_arm_wrist_rotate': '{:1.4f}'.format(right_arm_wrist_rotate), 
                'right_arm_claw': '{:1.4f}'.format(right_arm_claw), 

                'l_arm': 'L:',      # make file easier for humans to read
                'left_arm_shoulder_rotate': '{:1.4f}'.format(left_arm_shoulder_rotate), 
                'left_arm_shoulder_lift': '{:1.4f}'.format(left_arm_shoulder_lift), 
                'left_arm_elbow_rotate': '{:1.4f}'.format(left_arm_elbow_rotate), 
                'left_arm_elbow_bend': '{:1.4f}'.format(left_arm_elbow_bend), 
                'left_arm_wrist_rotate': '{:1.4f}'.format(left_arm_wrist_rotate), 
                'left_arm_claw': '{:1.4f}'.format(left_arm_claw), 

                # user will edit CSV to add parameters for things like text to speak, light state, etc.
                'param1': "",  
                'param2': ""
                })

        self.marker_flag_set = False
        
if __name__=='__main__':

    total = len(sys.argv)
    cmdargs = str(sys.argv)
    if total > 1:
        option = sys.argv[1].lower()
        if "head" in option:
            print 'RECORDING HEAD ONLY'
        elif "left" in option:
            print 'RECORDING LEFT ARM ONLY'
        elif "right" in option:
            print 'RECORDING RIGHT ARM ONLY'
        elif "all" in option:
            print 'RECORDING ALL JOINTS'
        elif "marker" in option:
            print 'RECORDING MARKER ONLY'
        else:
            print 'USAGE: specify one of: all, head, right, left (for arms)'
            sys.exit()

        RecordServoPositions(option)
        rospy.spin()

    else:
        print 'USAGE: specify one of: all, head, right, left (for arms)'
        #sys.exit()



