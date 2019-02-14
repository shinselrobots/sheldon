#!/usr/bin/env python
#test client for joint_states_listener

import roslib
roslib.load_manifest('joint_states_listener')
import rospy
from joint_states_listener.srv import ReturnJointStates
import time
import sys
# -------------------------------------------------------------------------
# TODO: use this instead of hard-coded values here! import servo_joint_list

head_joints = [
    'head_sidetilt_joint',
    'head_pan_joint',
    'head_tilt_joint',
    ]

right_arm_joints = [
    'right_arm_shoulder_rotate_joint',
    'right_arm_shoulder_lift_joint',
    'right_arm_elbow_rotate_joint',
    'right_arm_elbow_bend_joint',
    'right_arm_wrist_bend_joint',
    'right_arm_wrist_rotate_joint',
    'right_arm_gripper_finger_joint',
    ]

left_arm_joints = [
    'left_arm_shoulder_rotate_joint',
    'left_arm_shoulder_lift_joint',
    'left_arm_elbow_rotate_joint',
    'left_arm_elbow_bend_joint',
    'left_arm_wrist_bend_joint',
    'left_arm_wrist_rotate_joint',
    'left_arm_gripper_finger_joint',
    ]

chest_camera_joints = [
    'chest_camera_tilt_joint',
]

leg_joints = [
    'knee_bend_joint',
    'waist_bend_joint',
    ]

all_joints = head_joints + chest_camera_joints + right_arm_joints + left_arm_joints

right_arm_kinematic_joints =  leg_joints + right_arm_joints
left_arm_kinematic_joints =  leg_joints + left_arm_joints
# -------------------------------------------------------------------------



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
        (position, velocity, effort) = call_return_joint_states(head_joints)
        print "position:", pplist(position)
        print "velocity:", pplist(velocity)
        print "effort:", pplist(effort)
        time.sleep(1)
