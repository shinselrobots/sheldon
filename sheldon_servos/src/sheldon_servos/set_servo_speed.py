#!/usr/bin/env python

import sys
import roslib
roslib.load_manifest('sheldon_servos')
import rospy, time
from dynamixel_controllers.srv import TorqueEnable, SetTorqueLimit, SetSpeed
from servo_joint_list import all_servo_joints, head_joints, right_arm_joints, left_arm_joints

# speed value in Radians/Second. Max speed: MX106 ~ 5.24 (50RPM), MX64 ~ 6.6, RX28 ~ 8.3 (80RPM)
# For our purposes, we usually assume max speed of 5.0, so all servos run at predictable speed.

class SetServoSpeed():
    def __init__(self, speed, joints):
        rospy.loginfo('SetServoSpeed to %1.4f rad/sec:' %speed)
        #rospy.loginfo("Servo Speed to  (press ctrl-c to cancel at anytime)")

        speed_services = list()

        for controller in sorted(joints):            
            speed_service = '/' + controller + '/set_speed'
            print('  ' + speed_service)
            rospy.wait_for_service(speed_service)  
            speed_services.append(rospy.ServiceProxy(speed_service, SetSpeed))
            
        # Set the speed
        #print '  setting servo speeds to ', speed
        for set_speed in speed_services:
            set_speed(speed)

        print("SetServoSpeed complete.")

        
class SetSingleServoSpeed():
    def __init__(self, speed, servo_controller):
        # input: a servo controller string, for example: 'right_arm_shoulder_rotate_joint'

        rospy.loginfo('SetSingleServoSpeed to %1.4f rad/sec:' %speed)
        speed_service = '/' + servo_controller + '/set_speed'
        print('  ' + speed_service)

        rospy.wait_for_service(speed_service)  
        set_speed = rospy.ServiceProxy(speed_service, SetSpeed)
            
        # Set the speed
        set_speed(speed)

        print("SetSingleServoSpeed complete.")

        
if __name__=='__main__':

    total = len(sys.argv)
    cmdargs = str(sys.argv)
    joints = all_servo_joints

    if total > 2:
        option = sys.argv[1].lower()
        if "all_servo_joints" in option:
            print 'Setting all_servo_joints'
            joints = all_servo_joints
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
            print 'USAGE: specify one of: all_servo_joints, head_joints, right_arm_joints, left_arm_joints'
            sys.exit()

        speed = float(sys.argv[2])

        try:
            SetServoSpeed(speed, joints)
            rospy.loginfo("*** Set Speed Done ***")
        except rospy.ROSInterruptException:
            rospy.loginfo("Oops! Exception occurred while trying to set speed.") 

    else:
        print 'USAGE: set_servo_speed.py <joint_group> (all_servo_joints, head_joints, right_arm_joints, left_arm_joints)  <speed>  (0.0 - 5.0)'
        #sys.exit()






