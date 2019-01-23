#!/usr/bin/env python

import sys
import roslib
import rospy, time

import tf
from tf import TransformListener
#import moveit_commander
#from moveit_commander import MoveGroupCommander
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import geometry_msgs.msg


class GetJointPose():
    def __init__(self, target_link):
                       
        rospy.init_node('GetJointPose')
        tf_listener = TransformListener()
        base_link = "base_link"

        print "waiting for transform"
        tf_listener.waitForTransform (target_link, base_link, rospy.Time(), rospy.Duration(4.0))
        print "done waiting"

        if not tf_listener.frameExists("base_link"):
            print "ERROR NO FRAME base_link"
            return
        
        elif not tf_listener.frameExists(target_link):
            print "ERROR NO FRAME" +  target_link
            return
        
        else:
            t = tf_listener.getLatestCommonTime("/base_link", target_link)
            joint_pose = geometry_msgs.msg.PoseStamped()
            joint_pose.header.frame_id = target_link
            joint_pose.pose.orientation.w = 1.0    # Neutral orientation
            pose_from_base = tf_listener.transformPose("/base_link", joint_pose)
            print "Position from " + base_link + " to " + target_link
            print pose_from_base



if __name__=='__main__':

    total = len(sys.argv)
    #cmdargs = str(sys.argv)
    #joints = all_joints

    if total > 1:
        option = sys.argv[1].lower()

        try:
            GetJointPose(option)
        except rospy.ROSInterruptException:
            rospy.loginfo("Oops! Exception occurred.") 

    else:
        print 'USAGE: get_link_xyz.py <link_name> : (use check_urdf <path to robot .urdf>)'
        print 'Example: Sheldon right arm:'
        print 'right_arm_shoulder_rotate_servo_link'
        print 'right_arm_shoulder_rotate_link'
        print 'right_arm_shoulder_lift_servo_link'
        print 'right_arm_upper_link'
        print 'right_arm_elbow_rotate_servo_link'
        print 'right_arm_elbow_rotate_link'
        print 'right_arm_elbow_bend_servo_link'
        print 'right_arm_elbow_bend_link'
        print 'right_arm_lower_link'
        print 'right_arm_wrist_bend_servo_link'
        print 'right_arm_wrist_bend_link'
        print 'right_arm_wrist_rotate_servo_link'
        print 'right_arm_wrist_rotate_link'
        print 'right_arm_gripper_body_link'
        print 'right_arm_gripper_finger_link1'
        print 'right_arm_gripper_finger_link2'
        print 'right_arm_gripper_link'
        #sys.exit()





