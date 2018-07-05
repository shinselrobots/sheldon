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

        for controller in sorted(joints):            
            print('  /' + controller)
            torque_enable_service = '/' + controller + '/torque_enable'
            set_torque_limit_service = '/' + controller + '/set_torque_limit'

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
    def __init__(self, torque, servo_controller):
        # input: a servo controller string, for example: 'right_arm_shoulder_rotate_controller'
                       
        rospy.loginfo('SetSingleServoTorque to %1.4f: (1.0 = max)' %torque)
        print('  /' + servo_controller)

        torque_enable_service = '/' + servo_controller + '/torque_enable'
        torque_limit_service = '/' + servo_controller + '/set_torque_limit'

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

 

