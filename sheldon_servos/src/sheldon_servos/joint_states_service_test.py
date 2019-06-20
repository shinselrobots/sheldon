#!/usr/bin/env python
# test client for joint_states_listener

import roslib
import rospy
from sheldon_servos.srv import ReturnJointStates
import time
import sys
# -------------------------------------------------------------------------
from servo_joint_list import *


def call_return_joint_states(joint_names):
    rospy.wait_for_service("return_joint_states")
    try:
        s = rospy.ServiceProxy("return_joint_states", ReturnJointStates)
        resp = s(joint_names)
    except rospy.ServiceException, e:
        print "error when calling return_joint_states: %s"%e
        sys.exit(1)
    for (ind, joint_name) in enumerate(joint_names):
        if(not resp.found[ind]):
            print "joint %s not found!"%joint_name
    return (resp.position, resp.velocity, resp.effort)


#pretty-print list to string
def pplist(list):
    return ' '.join(['%2.3f'%x for x in list])


#print out the positions, velocities, and efforts of the right arm joints
if __name__ == "__main__":

    while(1):
        print '------------------------------'
        # selecdt joints to test:  right_arm_kinematic_joints, all_joints, etc.
        (position, velocity, effort) = \
            call_return_joint_states(right_arm_kinematic_joints)
        print "position:", pplist(position)
        print "velocity:", pplist(velocity)
        print "effort:", pplist(effort)
        time.sleep(1)
