#!/usr/bin/env python

"""
    set_head_torque.py 
    
    Usage: set_head_torque.py <float value>
    Values:  0.00 - 1.00 represent 0 - 100 percent.

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

class SetTorque():
    def __init__(self, torque):
                        
        torque_services = list()
        set_torque_limit_services = list()


        for controller in sorted(right_arm_joints):            
            torque_service = '/' + controller + '/torque_enable'
            print('  waiting for service: ' + torque_service)

            rospy.wait_for_service(torque_service)  
            torque_services.append(rospy.ServiceProxy(torque_service, TorqueEnable))
            
            set_torque_limit_service = '/' + controller + '/set_torque_limit'
            print('  waiting for service: ' + set_torque_limit_service)
            rospy.wait_for_service(set_torque_limit_service)  
            set_torque_limit_services.append(rospy.ServiceProxy(set_torque_limit_service, SetTorqueLimit))
            
        # Set the torque limit to a moderate value
        print '  setting torque limits to ', torque
        for set_torque_limit in set_torque_limit_services:
            set_torque_limit(torque)

        # Enable torque.
        for torque_enable in torque_services:
            if torque == 0.0:
                torque_enable(False) # if 0, set torque completely off!
            else:
                torque_enable(True)

        print("Done setting torque.")

        
if __name__=='__main__':

    rospy.init_node('set_right_arm_torque')
    default_dynamixel_torque = rospy.get_param('~default_dynamixel_torque', 0.5)


    total = len(sys.argv)
    cmdargs = str(sys.argv)
    #print ("The total numbers of args passed to the script: %d " % total)
    #print ("Args list: %s " % cmdargs)
    #print ("Script name: %s" % str(sys.argv[0]))
    #for i in xrange(total):
    #    print ("Argument # %d : %s" % (i, str(sys.argv[i])))

    if total > 1:
        newTorque = float(sys.argv[1])
        print 'Setting Right Arm Torque to: ',  newTorque
    else:
        print 'USAGE: torque value 0.00 - 1.00 (0 to 100 percent)'
        newTorque = default_dynamixel_torque
        print 'Setting Torque to default_dynamixel_torque: ',  newTorque 


    SetTorque(newTorque)
