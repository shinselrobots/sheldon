#!/usr/bin/env python

"""

Display the current joint positions of the arm in kinematic order of the links

"""

import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class GetJointStates:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the ROS node
        rospy.init_node('get_joint_states', anonymous=True)
                
        # Initialize the MoveIt! commander for the right arm
        arm = MoveGroupCommander('right_arm')
        
        # Get the end-effector link
        end_effector_link = arm.get_end_effector_link()
        rospy.loginfo("End effector: %s" % end_effector_link) 
 
        planning_frame = arm.get_planning_frame()

        
        # Joints are stored in the order they appear in the kinematic chain
        joint_names = arm.get_active_joints()
        
        joint_names = [
            'right_arm_shoulder_rotate_joint', 'right_arm_shoulder_lift_joint', 
            'right_arm_elbow_rotate_joint',    'right_arm_elbow_bend_joint',
            'right_arm_wrist_bend_joint',      'right_arm_wrist_rotate_joint']


        # Display the joint names
        #rospy.loginfo("Joint names:\n"  + str(joint_names) + "\n")
        
        # Get the current joint angles
        joint_values = arm.get_current_joint_values()
        
        # Display the joint values
        rospy.loginfo("Joint values:\n"  + str(joint_values) + "\n")
        
        # Get the end-effector pose
        ee_pose = arm.get_current_pose(end_effector_link)

        orientation = ee_pose.pose.orientation
        ox = orientation.x
        oy = orientation.y
        oz = orientation.z
        ow = orientation.w
                       
        euler_pose = euler_from_quaternion([ow, ox, oy, oz])
        #euler_pose = euler_from_quaternion([0.0, 0.0, 0.0, 1.0])
        
        # Display the end-effector pose
        rospy.loginfo("End effector pose:\n" + str(ee_pose))
        rospy.loginfo("RPY?:\n" + str(euler_pose))
        
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    GetJointStates()
    
