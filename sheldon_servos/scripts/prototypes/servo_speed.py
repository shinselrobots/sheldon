#!/usr/bin/env python

import argparse
import sys
import roslib
roslib.load_manifest('sheldon_servos')
import rospy, time
#from dynamixel_controllers.srv import TorqueEnable, SetTorqueLimit, SetSpeed
from sheldon_servos.servo_joint_list import all_joints, head_joints, right_arm_joints, left_arm_joints

#from sheldon_servos.standard_servo_positions import *
from sheldon_servos.set_servo_speed import *
from sheldon_servos.set_servo_torque import *

# ===================================================================
# Set speed value in Radians/Second. Max speed: MX106 ~ 5.24 (50RPM), MX64 ~ 6.6, RX28 ~ 8.3 (80RPM)
# For our purposes, we usually assume max speed of 5.0, so all servos run at predictable speed.
# ===================================================================


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description="set speed of: all, head, right_arm, or left_arm joints")
    parser.add_argument("-s", "--speed", required=True, help="speed in rad/deg, 0.5 suggested", dest="speed")
    parser.add_argument("-j", "--joint", required=True, help="one of: all, head, right_arm, left_arm, or servo name (eg. 'right_arm_shoulder_rotate')", dest="joints")

    rospy.init_node('set_servo_speed_command')

    single_servo = False
    args = parser.parse_args()
    _speed = float(args.speed)

    if('all' == args.joints):
        _joints = all_joints
    elif('head' == args.joints):
        _joints = head_joints
    elif('right_arm' == args.joints):
        _joints = right_arm_joints
    elif('left_arm' == args.joints):
        _joints = left_arm_joints
    else:
        # Assume it's a specific servo
        print("    ===> Assuming single servo.  If bad name, this will hang.  Ctrl-C to exit.")
        _joints = args.joints + '_controller'
        single_servo = True

    if(single_servo):
        SetSingleServoSpeed(_speed, _joints)
    else:
        SetServoSpeed(_speed, _joints)


