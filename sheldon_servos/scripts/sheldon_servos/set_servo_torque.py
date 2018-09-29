#!/usr/bin/env python

import sys
import roslib
roslib.load_manifest('sheldon_servos')
import rospy, time
from dynamixel_controllers.srv import TorqueEnable, SetTorqueLimit, SetSpeed
from servo_joint_list import all_joints, head_joints, right_arm_joints, left_arm_joints

class SetServoTorque():
    def __init__(self, torque, joints):
                       
        rospy.loginfo('SetServoTorque to %1.4f: (1.0 = max)' %torque)

        torque_enable_services = list()
        set_torque_limit_services = list()

        for joint in sorted(joints):            
            print('  /' + joint)
            torque_enable_service = '/' + joint + '/torque_enable'
            set_torque_limit_service = '/' + joint + '/set_torque_limit'

            rospy.wait_for_service(torque_enable_service)  
            torque_enable_services.append(rospy.ServiceProxy(torque_enable_service, TorqueEnable))
            
            rospy.wait_for_service(set_torque_limit_service)  
            set_torque_limit_services.append(rospy.ServiceProxy(set_torque_limit_service, SetTorqueLimit))

        if torque == 0.0:
            # Turn off torque
            #print("  turning off torque...")
            for torque_enable in torque_enable_services:
                torque_enable(False)
            #print("  torque off")

        else:
            # Set the torque limit to a requested value
            #print '  setting torque limits to ', torque
            for set_torque_limit in set_torque_limit_services:
                set_torque_limit(torque)

            # Enable torque.
            for torque_enable in torque_enable_services:
                torque_enable(True)

        print("  SetServoTorque complete.")


class SetSingleServoTorque():
    def __init__(self, torque, servo_joint):
        # input: a servo joint string, for example: 'right_arm_shoulder_rotate_joint'
                       
        rospy.loginfo('SetSingleServoTorque to %1.4f: (1.0 = max)' %torque)
        print('  /' + servo_joint)

        torque_enable_service = '/' + servo_joint + '/torque_enable'
        torque_limit_service = '/' + servo_joint + '/set_torque_limit'

        rospy.wait_for_service(torque_enable_service)  
        enable_torque = rospy.ServiceProxy(torque_enable_service, TorqueEnable)

        rospy.wait_for_service(torque_limit_service)  
        set_torque_limit = rospy.ServiceProxy(torque_limit_service, SetTorqueLimit)

        if torque == 0.0:
            # Turn off torque
            #print("turning off torque...")
            enable_torque(False)
            #print("torque off")

        else:
            # Set the torque limit to a requested value
            # print '  setting servo torque to ', torque
            set_torque_limit(torque)
            enable_torque(True)

        print("  SetSingleServoTorque complete.")

if __name__=='__main__':

    total = len(sys.argv)
    cmdargs = str(sys.argv)
    joints = all_joints

    if total > 2:
        option = sys.argv[1].lower()
        if "all_joints" in option:
            print 'Setting all_joints'
            joints = all_joints
        elif "head_joints" in option:
            print 'Setting head_joints'
            joints = head_joints
        elif "right_arm_joints" in option:
            print 'Setting right_arm_joints'
            joints = right_arm_joints
        elif "left_arm_joints" in option:
            print 'Setting left_arm_joints'
            joints = left_arm_joints
        else:
            print 'USAGE: specify one of: all_joints, head_joints, right_arm_joints, left_arm_joints'
            sys.exit()

        torque = float(sys.argv[2])

        try:
            SetServoTorque(torque, joints)
            rospy.loginfo("*** Set Torque Done ***")
        except rospy.ROSInterruptException:
            rospy.loginfo("Oops! Exception occurred while trying to set torque.") 

    else:
        print 'USAGE: set_servo_torque.py <joint_group> <torque_value>   where group is one of: all_joints, head_joints, right_arm_joints, left_arm_joints'
        #sys.exit()










