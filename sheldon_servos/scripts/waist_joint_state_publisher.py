#!/usr/bin/env python

"""
    waist_joint_state_publisher.py
    Convert waist position and publish on Publish the dynamixel_controller joint states on the /joint_states topic
    
"""
import rospy
import logging
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

def callback(data):

    # calculate joint positions
    kneePositionRadians = data.data
    waistPositionRadians = data.data * 2.0 
    # TODO - this is approximate!  Need to work out exact formula!

    # todo - calculate velocity (radians/sec), 
    # using time and change since last msg
 
    # Construct message & publish joint states
    msg = JointState()
    msg.name = []
    msg.position = []
    msg.velocity = []
    msg.effort = []

    # Knee Joint   
    msg.name.append("knee_joint")
    msg.position.append(kneePositionRadians)
    msg.velocity.append(0.0)
    msg.effort.append(0.0)

    # Waist Joint   
    msg.name.append("waist_joint")
    msg.position.append(waistPositionRadians)
    msg.velocity.append(0.0)
    msg.effort.append(0.0)
      
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'base_link'
    pub_joint_states.publish(msg) 
   

def waistJointStatePublisher():
    rospy.init_node('waist_joint_state_publisher', anonymous=True)
    rospy.Subscriber("/waist_current_position", Float32, callback)
    rospy.spin()


if __name__ == '__main__':
    # convert waist position messages to standard "JointState" messages
    pub_joint_states = rospy.Publisher('/joint_states', JointState, queue_size=2, latch=True)
    try:
        s = waistJointStatePublisher()
        rospy.spin()
    except rospy.ROSInterruptException: pass


