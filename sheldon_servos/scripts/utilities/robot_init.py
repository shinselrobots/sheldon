#!/usr/bin/env python

import roslib
roslib.load_manifest('sheldon_servos')
import rospy, time
from dynamixel_controllers.srv import TorqueEnable, SetTorqueLimit, SetSpeed
from sheldon_servos.servo_joint_list import all_joints, head_joints, right_arm_joints
from sheldon_servos.standard_servo_positions import *
from std_msgs.msg import Empty

class WakeupPosition():
    def __init__(self):

        rospy.init_node('wakeup_position')

        pub_waist = rospy.Publisher('/waist_calibrate', Empty, queue_size=1)        
               
        speed_services = list()   
        set_torque_limit_services = list()

        print '  loop through servo services...'
            
        for controller in sorted(all_joints):            
           
            set_torque_limit_service = '/' + controller + '/set_torque_limit'
            rospy.wait_for_service(set_torque_limit_service)  
            set_torque_limit_services.append(rospy.ServiceProxy(set_torque_limit_service, SetTorqueLimit))
            
            speed_service = '/' + controller + '/set_speed'
            rospy.wait_for_service(speed_service)  
            speed_services.append(rospy.ServiceProxy(speed_service, SetSpeed))
        

        # Set to medium speed
        speed = 1.0
        print '  setting speeds to ', speed
        for set_speed in speed_services:
            set_speed(speed)

        # Set medium torque limit
        torque = 0.60  # percent of max
        print '  setting torque limits to ', torque
        for set_torque_limit in set_torque_limit_services:
            set_torque_limit(torque)

        print '  moving arms to ready position...'
        all_home()
        time.sleep(3.0)  # allow time for servos to move to position

        # initialize waist position
        print("  calibrating waist Position...")
        pub_waist.publish()

        print("Done moving to wakeup position.")

        
if __name__=='__main__':
    WakeupPosition()

