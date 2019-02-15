#!/usr/bin/env python

import argparse
import sys
import roslib
roslib.load_manifest('sheldon_servos')
import rospy, time
#from dynamixel_controllers.srv import TorqueEnable, SetTorqueLimit, SetSpeed
from sheldon_servos.servo_joint_list import all_servo_joints, head_joints, right_arm_joints, left_arm_joints

#from sheldon_servos.standard_servo_positions import *
from sheldon_servos.set_servo_speed import *
from sheldon_servos.set_servo_torque import *

# ===================================================================
# Set torque value in percent ( 0.0 to 1.0)
# ===================================================================


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description="set torque of: all, head, right_arm, left_arm joints, or single servo")
    parser.add_argument("-t", "--torque", required=True, help="torque from 0.0 to 1.0, 0.5 suggested", dest="torque")
    parser.add_argument("-j", "--joint", required=True, help="one of: all, head, right_arm, left_arm, or servo name (eg. 'right_arm_shoulder_rotate')", dest="joints")

    rospy.init_node('set_servo_torque_command')

    single_servo = False
    args = parser.parse_args()
    _torque = float(args.torque)

    if('all' == args.joints):
        _joints = all_servo_joints
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
        SetSingleServoTorque(_torque, _joints)
    else:
        SetServoTorque(_torque, _joints)


