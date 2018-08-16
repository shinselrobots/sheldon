#!/usr/bin/env python
# WORKS WITH SHELDON!

import roslib
import rospy
import actionlib
import time
from std_msgs.msg import Float64
import trajectory_msgs.msg 
import control_msgs.msg  
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal



class Joint:
        def __init__(self, meta_controller_name):
            #meta_controller_name is one of: 'head', 'left_arm', 'right_arm'
            self.name = meta_controller_name           
            self.jta = actionlib.SimpleActionClient('/'+self.name+'_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            rospy.loginfo('Waiting for joint trajectory action')
            self.jta.wait_for_server()
            rospy.loginfo('Found joint trajectory action!')

            if self.name == 'head':
                rospy.loginfo('Moving Head...')
                self.joint_names = ['head_sidetilt_joint','head_tilt_joint','head_pan_joint']

            elif self.name == 'left_arm':
                rospy.loginfo('Moving Left Arm...')
                self.joint_names = [
                    'left_arm_shoulder_rotate_joint',  'left_arm_shoulder_lift_joint',
                    'left_arm_elbow_rotate_joint',     'left_arm_elbow_bend_joint', 
                    'left_arm_wrist_rotate_joint']

            else:
                rospy.loginfo('Moving Right Arm...')
                self.joint_names = [
                    'right_arm_shoulder_rotate_joint', 'right_arm_shoulder_lift_joint', 
                    'right_arm_elbow_rotate_joint',    'right_arm_elbow_bend_joint',
                    'right_arm_wrist_rotate_joint'] 


            
        def move_joint(self, angles, duration):
            goal = FollowJointTrajectoryGoal()                  
            goal.trajectory.joint_names = self.joint_names
            point = JointTrajectoryPoint()
            point.positions = angles
            point.time_from_start = rospy.Duration(duration)                   
            goal.trajectory.points.append(point)
            self.jta.send_goal_and_wait(goal)
              

def main():
            move_duration = 0.5  # time in seconds for move to complete!
            #head_group = Joint('head')
            #head_group.move_joint([0.25, 0.5, 0.5], move_duration)
            #head_group.move_joint([-0.25, -0.5, -0.5], move_duration*2.0)
            #head_group.move_joint([0.0, 0.0, 0.0], move_duration)

            right_arm_group = Joint('right_arm')
            rospy.loginfo('Moving Position 1')
            right_arm_group.move_joint([0.7, 0.13, 0.0, 1.5, 0.0], move_duration) # extend out
            time.sleep(2.0)
            rospy.loginfo('Moving Postion 2 (Home)')
            right_arm_group.move_joint([-0.49, 0.13, 0.0, 2.15, 0.0], move_duration) # home
            rospy.loginfo('DONE')

            left_arm_group = Joint('left_arm')
            rospy.loginfo('Moving Position 1')
            left_arm_group.move_joint([0.7, 0.13, 0.0, 1.5, 0.0], move_duration) # extend out
            time.sleep(2.0)
            rospy.loginfo('Moving Postion 2 (Home)')
            left_arm_group.move_joint([-0.49, 0.13, 0.0, 2.15, 0.0], move_duration) # home
            rospy.loginfo('DONE')

                        
if __name__ == '__main__':
      rospy.init_node('joint_position_tester')
      main()
