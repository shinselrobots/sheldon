#!/usr/bin/env python

"""

Uses TF to directly get the pose of right_arm_gripper_link
No MoveIt needed!

"""

import rospy, sys, tf
from tf import TransformListener
#import moveit_commander
#from moveit_commander import MoveGroupCommander
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import geometry_msgs.msg


base_link = "base_link"
#target_link = "right_arm_shoulder_lift_servo_link"
#target_link = "right_arm_upper_link"
target_link = "right_arm_gripper_link"


class GetJointPose:
    def __init__(self):
    
        rospy.init_node('GetJointPose')
        tf_listener = TransformListener()

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
    

if __name__ == "__main__":
    GetJointPose()
    
