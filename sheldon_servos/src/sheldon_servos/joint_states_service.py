#!/usr/bin/env python
# spins off a thread to listen for joint_states messages
# and provides the same information (or subsets of) as a service
# from sample at http://wiki.ros.org/pr2_controllers/Tutorials/Getting%20the%20current%20joint%20angles

import roslib
import rospy
import logging
from sensor_msgs.msg import JointState
from sheldon_servos.srv import *
import threading
from servo_joint_list import *

#holds the latest states obtained from joint_states messages
class LatestJointStates:

    def __init__(self):
        rospy.init_node('joint_states_listener')
        self.lock = threading.Lock()

        # joint_name index will be used to index into all other lists
        # because callbacks for joints come in random order
        self.name = sorted(all_joints) 
        self.position = []
        self.velocity = []
        self.effort = []
        self.thread = threading.Thread(target=self.joint_states_listener)
        self.thread.start()

        print "=========================================="
        print self.name
        print "initializing defaults"

        for i in range(len(self.name)):
            print str(i) + " :  [" + self.name[i] + "]"
            
            self.position.append(0.0)
            self.velocity.append(0.0)
            self.effort.append(0.0)

        print "initializing done, starting service..."
        s = rospy.Service('return_joint_states', ReturnJointStates, self.return_joint_states)
        

    # thread function: listen for joint_states messages
    def joint_states_listener(self):
        rospy.Subscriber('joint_states', JointState, self.joint_states_callback)
        rospy.spin()


    # callback function: when a joint_states message arrives, save the values
    # values come in random order, so for each item, find it's position in
    # this service's array, and save the values.
    def joint_states_callback(self, msg):

        ignored_joints = {'left_arm_gripper_finger_joint2', 'right_arm_gripper_finger_joint2'}          
        self.lock.acquire()

        #print "=========================================="
        for i in range(len(msg.name)):
        
            if msg.name[i] in ignored_joints:
                continue
        
            #print "Callback: got updates for: "
            #print str(i) + ": " + msg.name[i] + " " \
            #    + str(msg.position[i])  + " " 

            # find the joint name in our list
            joint_name = msg.name[i]
            #print "requesting index for [" + joint_name + "]"
            try:            
                j = self.name.index(joint_name) 
                #print "got index: [" + str(j) + "]"

            except Exception:
                sys.exc_clear() # clear the exception (python2)
                rospy.logwarn("ERROR: Unknown Joint reported in /joint_states msg: [" \
                    + joint_name + "]")     


            # save values (skip any that are not supplied)
            #print "saving values for [" + joint_name + "]"
            self.position[j] = msg.position[i]
            self.velocity[j] = 0.0
            self.effort[j] = 0.0

                
            try:
                # NOTE: velocity and effort are not published when using SIMULATION
                # so handle the exception here  
                self.velocity[j] = msg.velocity[i]
                self.effort[j] = msg.effort[i]

            except Exception:
                sys.exc_clear() # clear the exception (python2)
                #print "ignoring velocity and effort"
            

        self.lock.release()


    #returns (found, position, velocity, effort) for the joint joint_name 
    #(found is 1 if found, 0 otherwise)
    def return_joint_state(self, joint_name):

        #print "DGB: ===== return_joint_state() called ====== "

        #return info for this joint
        self.lock.acquire()
        #print "DGB: looking for name... "
        
        if joint_name in self.name:
            #print "DGB: found name [" + joint_name + "], getting index..." 
            
            index = self.name.index(joint_name)
            #print "DGB: got index: " + str(index)

            #print "DGB: getting position..."
            position = self.position[index]
            #print "DGB: getting velocity..."
            velocity = self.velocity[index]
            #print "DGB: getting effort..."
            effort = self.effort[index]

        #unless it's not found
        else:
            rospy.logerr("Joint %s not found!", (joint_name,))
            self.lock.release()
            return (0, 0., 0., 0.)
        self.lock.release()
        return (1, position, velocity, effort)


    #server callback: returns arrays of position, velocity, and effort
    #for a list of joints specified by name
    def return_joint_states(self, req):
        #print "DGB: ===== return_joint_stateS() called ====== "
        joints_found = []
        positions = []
        velocities = []
        efforts = []
        for joint_name in req.name:
            #print "DGB: getting info for joint" + joint_name
            (found, position, velocity, effort) = self.return_joint_state(joint_name)
            joints_found.append(found)
            positions.append(position)
            velocities.append(velocity)
            efforts.append(effort)
        #print "DGB: Calling ReturnJointStatesResponse()"
        return ReturnJointStatesResponse(joints_found, positions, velocities, efforts)


#run the server
if __name__ == "__main__":

    latestjointstates = LatestJointStates()

    #print "joints_states_service started, waiting for queries"
    rospy.loginfo("joints_states_service started, waiting for queries")

    rospy.spin()
