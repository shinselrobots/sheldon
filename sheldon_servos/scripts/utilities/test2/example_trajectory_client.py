#!/usr/bin/env python
import roslib
roslib.load_manifest('sheldon_servos')

import rospy
import actionlib
from std_msgs.msg import Float64
import trajectory_msgs.msg 
import control_msgs.msg  
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal



class Joint:
        def __init__(self, meta_controller_name):
            #meta_controller_name should be 'head', 'left_arm', 'right_arm'
            self.name = meta_controller_name           
            self.jta = actionlib.SimpleActionClient('/'+self.name+'_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            rospy.loginfo('Waiting for joint trajectory action')
            self.jta.wait_for_server()
            rospy.loginfo('Found joint trajectory action!')
            #if self.name == 'head':
            self.joint_names = ['head_sidetilt_joint','head_tilt_joint','head_pan_joint']
            #elif self.name == 'left_arm':
            #    self.joint_names = ['left_shoulder_rotate_joint','left_shoulder_lift_joint','left_elbow_rotate_joint']
            #else:
            #    self.joint_names = ['right_shoulder_rotate_joint','right_shoulder_lift_joint','right_elbow_rotate_joint']

            
        def move_joint(self, angles, duration):
            goal = FollowJointTrajectoryGoal()                  
            goal.trajectory.joint_names = self.joint_names
            point = JointTrajectoryPoint()
            point.positions = angles
            point.time_from_start = rospy.Duration(duration)                   
            goal.trajectory.points.append(point)
            self.jta.send_goal_and_wait(goal)
              

def main():
            group = Joint('head')
            group.move_joint([0.25, 0.5, 0.5], 0.5)
            group.move_joint([-0.25, -0.5, -0.5], 1.0)
            group.move_joint([0.0, 0.0, 0.0], 0.5)
            #arm.move_joint([6.28,3.14,6.28])

                        
if __name__ == '__main__':
      rospy.init_node('joint_position_tester')
      main()
