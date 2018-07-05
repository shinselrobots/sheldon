#!/usr/bin/env python

"""
    set_head_speed.py 
    
    Usage: set_head_speed.py <float value>
    Values for RX-28:
       10.0 full speed?
        5.0 fast
        3.0 moderate
        1.0 med-slow
        0.5 slow
    Values are clear from docs/code... should be radian/second, but seems way off from that?

    LICENSE
    Based upon sample code from: ROS BY EXAMPLE Copyright 2012 Patrick Goebel    
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html

"""

import sys
import roslib
roslib.load_manifest('sheldon_servos')
import rospy, time
from dynamixel_controllers.srv import TorqueEnable, SetTorqueLimit, SetSpeed
from servo_joint_list import all_joints, head_joints, right_arm_joints

class SetServoSpeed():
    def __init__(self, speed):

        speed_services = list()

        for controller in sorted(head_joints):            
            speed_service = '/' + controller + '/set_speed'
            print('  waiting for service: ' + speed_service)
            rospy.wait_for_service(speed_service)  
            speed_services.append(rospy.ServiceProxy(speed_service, SetSpeed))
            
        # Set the speed
        print '  setting servo speeds to ', speed
        for set_speed in speed_services:
            set_speed(speed)

        print("Done setting speed.")

        
if __name__=='__main__':

    rospy.init_node('set_head_speed')
    default_dynamixel_speed = rospy.get_param('~default_dynamixel_speed', 0.5)

    total = len(sys.argv)
    cmdargs = str(sys.argv)
    #print ("The total numbers of args passed to the script: %d " % total)
    #print ("Args list: %s " % cmdargs)
    #print ("Script name: %s" % str(sys.argv[0]))
    #for i in xrange(total):
    #    print ("Argument # %d : %s" % (i, str(sys.argv[i])))

    if total > 1:
        newSpeed = float(sys.argv[1])
        print 'Setting speed to: ',  newSpeed
    else:
        print 'USAGE: speed value 0.00 - 1.00 (0 to 100 percent)'
        newSpeed = default_dynamixel_speed
        print 'Setting speed to default_dynamixel_speed: ',  newSpeed 


    SetServoSpeed(newSpeed)
