#!/usr/bin/env python

import roslib
#roslib.load_manifest('sheldon_servos')
import rospy, time
from dynamixel_controllers.srv import TorqueEnable, SetTorqueLimit, SetSpeed
from servo_joint_list import *
#all_servo_joints, head_joints, right_arm_joints, left_arm_joints

from standard_servo_positions import *

class SleepPosition():
    def __init__(self):
        rospy.init_node('sleep_position')
                
        speed_services = list()   
        torque_services = list()
        set_torque_limit_services = list()

        print 'loop through servo services...'
            
        for controller in sorted(left_arm_joints):            
            torque_service = '/' + controller + '/torque_enable'
            print('  waiting for service: ' + torque_service)

            rospy.wait_for_service(torque_service)  
            torque_services.append(rospy.ServiceProxy(torque_service, TorqueEnable))
            
            set_torque_limit_service = '/' + controller + '/set_torque_limit'
            rospy.wait_for_service(set_torque_limit_service)  
            set_torque_limit_services.append(rospy.ServiceProxy(set_torque_limit_service, SetTorqueLimit))
            
            speed_service = '/' + controller + '/set_speed'
            rospy.wait_for_service(speed_service)  
            speed_services.append(rospy.ServiceProxy(speed_service, SetSpeed))
        

        # Set to medium-slow speed
        speed = 0.8
        print '  setting speeds to ', speed
        for set_speed in speed_services:
            try:
                set_speed(speed)
            except:
                pass

        # Set medium torque limit
        torque = 0.50  # percent of max
        print '  setting torque limits to ', torque
        for set_torque_limit in set_torque_limit_services:
            try:
                set_torque_limit(torque)
            except:
                pass

        print 'moving to sleep position...'
        left_arm_sleep()
        time.sleep(3.0)  # allow time for servos to move to position

        # Relax all servos to give them a rest.
        print 'turning off torque...'
        for torque_enable in torque_services:
            try:
                torque_enable(False)
            except:
                pass

        print("Done moving to sleep position.")

        
if __name__=='__main__':
    SleepPosition()

