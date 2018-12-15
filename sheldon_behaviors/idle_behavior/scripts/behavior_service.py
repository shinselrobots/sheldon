#! /usr/bin/env python
# IDLE Behavior
# WARNING!  if person tracking is acting crazy, check that 
# the person tracker node is using the correct camera!!

import rospy
import actionlib
import behavior_common.msg
import time
import rospkg
import rosparam

from std_msgs.msg import Float64
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import random

from math import radians, degrees
import tf
import os, thread
from playsound import playsound

# for talking
import actionlib
import actionlib.action_client
import audio_and_speech_common.msg

# SHELDON Only
# from dynamixel_controllers.srv import TorqueEnable, SetServoTorqueLimit, SetSpeed
from sheldon_servos.servo_joint_list import head_joints
from sheldon_servos.head_servo_publishers import *

from sheldon_servos.standard_servo_positions import *
from sheldon_servos.set_servo_speed import *
from sheldon_servos.set_servo_torque import *

# TB2S ONLY
# from tb2s_pantilt.set_servo_speed import *
# from sheldon_servos.set_servo_torque import *

from body_tracker_msgs.msg import BodyTracker, BodyTrackerArray
from geometry_msgs.msg import PointStamped, Point, PoseStamped, Pose, Pose2D
#import geometry_msgs.msg

import tf

# TB2S ONLY
#pub_head_pan = rospy.Publisher('/head_pan_joint/command', Float64, queue_size=1)
#pub_head_tilt = rospy.Publisher('/head_tilt_joint/command', Float64, queue_size=1)


class BehaviorAction(object):

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, 
            behavior_common.msg.behaviorAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

        rospy.loginfo('%s: Initializing Python behavior service' % (self._action_name))

        # constants
        self.MAX_PAN = 1.5708 #  90 degrees
        self.MAX_TILT = 0.60  #  Limit vertical to assure good tracking
        self.DEADBAND_ANGLE = 0.0872665 # 5 deg deadband in middle to prevent osc
        self.DEFAULT_TILT_ANGLE = 0.00 # TB2S: tilt head up slightly to find people more easily
        self.NAME_TIMEOUT_SECS = 8.0
        self.HELLO_TIMEOUT_SECS = (10.0 * 60.0)
        
        #====================================================================
        # Behavior Settings

        # Load this behavior's parameters to the ROS parameter server
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path(self._action_name.strip("/")) # remove leading slash
        param_file_path = pkg_path + '/param/param.yaml'
        rospy.loginfo('%s: Loading Params from %s', self._action_name, param_file_path)
        paramlist = rosparam.load_file(param_file_path, default_namespace=self._action_name)
        for params, ns in paramlist:
            rosparam.upload_params(ns,params)

        # Get this behavior's parameters
        self.enable_body_tracking = rospy.get_param('~enable_body_tracking', True)
        rospy.loginfo('%s: PARAM: enable_body_tracking = %s', self._action_name, 
            self.enable_body_tracking)

        self.enable_random_head_movement = rospy.get_param('~enable_random_head_movement', True)
        rospy.loginfo('%s: PARAM: enable_random_head_movement = %s', self._action_name,
            self.enable_random_head_movement)

        self.head_pan_joint = rospy.get_param('~head_pan_joint', 'head_pan_joint')
        rospy.loginfo('%s: PARAM: head_pan_joint = %s', self._action_name,
            self.head_pan_joint)

        self.head_tilt_joint = rospy.get_param('~head_tilt_joint', 'head_tilt_joint')
        rospy.loginfo('%s: PARAM: head_tilt_joint = %s', self._action_name,
            self.head_tilt_joint)

        self.sound_effects_dir = rospy.get_param('~sound_effects_dir',
            '/home/system/catkin_robot/src/sheldon/sheldon_behaviors/resources/sounds/sound_effects')
        rospy.loginfo('%s: PARAM: sound_effects_dir = %s', self._action_name,
            self.sound_effects_dir)

        self.ding_path = os.path.join(self.sound_effects_dir, "ding.wav")
        rospy.loginfo("DBG: DING PATH: %s", self.ding_path)
        # playsound(self.ding_path) # test sound

        #====================================================================


        self.tracking = False
        self.joint_state = JointState() # for reading servo positions
        #self.astra_target = list()

        # Remember which person we are tracking
        #self.current_person_id = 0  # 0 = not tracking anyone
        #self.current_person_id_time = rospy.Time.now() # start timer
        self.named_person = ""
        self.named_person_id = 0
        self.named_person_time = rospy.Time.now() # start timer
        self.named_people_seen_today = {}

        # Publish current person by name, if recognized
        self.pub_current_user_name = rospy.Publisher('/person/name', String, queue_size=2)        

        rospy.loginfo("Waiting for speech server (press ctrl-c to cancel at anytime)")
        self.speech_client = actionlib.SimpleActionClient("/speech_service", \
            audio_and_speech_common.msg.speechAction)
        self.speech_client.wait_for_server()

        rospy.sleep(2)
        rospy.loginfo("testing speech")
        goal = audio_and_speech_common.msg.speechGoal(text_to_speak="testing speech system")
        self.speech_client.send_goal(goal)

       # Initialize tf listener
        #self.tf = tf.TransformListener()

        # Allow tf to catch up        
        #rospy.sleep(2)


    def joint_state_cb(self, msg):

        #rospy.loginfo("%s: joint_state_cb called", self._action_name)

        try:
            test = msg.name.index(self.head_pan_joint)
            self.joint_state = msg
        except:
            return

       # Get the current servo pan and tilt position
        try:
            current_pan = self.joint_state.position[
                self.joint_state.name.index(self.head_pan_joint)]
            current_tilt = self.joint_state.position[
                self.joint_state.name.index(self.head_tilt_joint)]
        except:
            return

        #rospy.loginfo("%s: joint_state_cb: Current Pan = %f, Tilt = %f", 
        #  self._action_name, current_pan, current_tilt)


    #====================================================================
    # 2D Tracking:  Message contains person horizontal (x) and vertical (y)
    #               position is relative to the depth image.  
    def position_cb(self, msg):
        #rospy.loginfo('%s: got position_cb message' % (self._action_name))

        # determine highest priority person to track
        # Message contains an array of people being tracked 
        # Note: tracking while stationary / idle is different from when person-following
  
        person_to_track_id = 0    
        person_to_track_index = 0    

        # Priority 1:  someone making a gesture         
        for i, person in enumerate(msg.detected_list):
            if person.gesture > -1:
                person_to_track_id = person.body_id
                person_to_track_index = i
                rospy.loginfo("DBG: GOT A GESTURE ") 
                break
 
        # Priority 2: Track the closest person 
        # this allows robot to change focus to different people, 
        # whoever is closest he will talk to. 
        # TODO? Track closest person that has face detected to reject tracking objects?              
        if person_to_track_id == 0:
            closest_person_distance = 100000
            closest_person_index = 0
            for i, person in enumerate(msg.detected_list):
                if person.position2d.z < closest_person_distance:
                    closest_person_distance = person.position2d.z
                    person_to_track_id = person.body_id
                    person_to_track_index = i

        if person_to_track_id != 0:

            # found someone to track
            person_info = msg.detected_list[person_to_track_index]
            if person_info.face_found == True:
                rospy.loginfo("%s: Face Found.  Name = " + person_info.name, self._action_name)

            if person_info.name != "":
                # We recognized this person!
                rospy.loginfo("%s: Tracking Name " + person_info.name, self._action_name)
                self.named_person = person_info.name
                self.named_person_id = person_to_track_id # associate ID with the name
                self.named_person_time = rospy.Time.now()
                self.pub_current_user_name.publish(self.named_person)
                
                # Say hello the first time we see someone in a while
                if self.named_person not in self.named_people_seen_today:
                    #current_time = rospy.get_rostime()
                    self.named_people_seen_today[self.named_person] = rospy.get_rostime() 
                    rospy.loginfo("=========== Saying Hello ===========")
                    goal = audio_and_speech_common.msg.speechGoal( \
                        text_to_speak = "hello " + self.named_person)
                    self.speech_client.send_goal(goal)

                else:
                    # we've said hello to this person, but lets see how long ago it was
                    time_since_hello = rospy.get_rostime() - self.named_people_seen_today[self.named_person]
                    rospy.loginfo("%s: DEBUG time_since_hello = %f", \
                        self._action_name, time_since_hello.to_sec())

                    if time_since_hello > rospy.Duration.from_sec(self.HELLO_TIMEOUT_SECS):
                        self.named_people_seen_today[self.named_person] = rospy.get_rostime() 
                        rospy.loginfo("=========== Saying Hello Again ===========")
                        goal = audio_and_speech_common.msg.speechGoal( \
                            text_to_speak = "hello again " + self.named_person)
                        self.speech_client.send_goal(goal)
                
            else:
            
                if self.named_person != "":
                    # We are tracking a specific person by name, but did not get a name this frame
                    if person_to_track_id != self.named_person_id:
                        # different user, clear the name
                        rospy.loginfo("%s: Lost user %s", self._action_name, self.named_person)
                        self.named_person = "" 
                        self.named_person_id = 0 
                        self.pub_current_user_name.publish(self.named_person)
                    else:
                        # still the same ID, but sometimes ID's don't get changed
                        time_since_last_name = rospy.Time.now() - self.named_person_time 
                        rospy.loginfo("%s: DEBUG time_since_last_name = %f", \
                            self._action_name, time_since_last_name.to_sec())

                        if time_since_last_name > rospy.Duration.from_sec(self.NAME_TIMEOUT_SECS):
                            rospy.loginfo("%s: User Name %s Timed out", self._action_name, self.named_person)
                            self.named_person = "" 
                            self.named_person_id = 0 
                            self.pub_current_user_name.publish(self.named_person)
                    
               
            #self.current_person_id = person_to_track_id
            #self.current_person_id_time = rospy.Time.now()

        
            # Track person
            # position in radians from center of camera lens
            delta_angle_x = person_info.position2d.x 
            delta_angle_y = person_info.position2d.y  

            # Uncomment this to debug
            # rospy.loginfo("%s: Tracking Person Index: %d, ID: %d x: %f y: %f", \
            #    self._action_name, person_to_track_index, person_to_track_id, delta_angle_x, delta_angle_y ) 
                
            # Get the current servo pan and tilt position
            try:
                current_pan = self.joint_state.position[
                    self.joint_state.name.index(self.head_pan_joint)]
                current_tilt = self.joint_state.position[
                    self.joint_state.name.index(self.head_tilt_joint)] * -1.0
            except:
                return

            #rospy.loginfo("%s: Body Tracker: Current Servo:  Pan = %f,  Tilt = %f", 
            #  self._action_name, current_pan, current_tilt)

            # add target position to current servo position
            pan_angle  = current_pan  + (delta_angle_x * 0.95) #shoot for less
            tilt_angle = current_tilt + (delta_angle_y * 0.95)
            # rospy.loginfo("%s: Body Tracker: Servo Command:  Pan = %f,  Tilt = %f", 
            #    self._action_name, pan_angle, tilt_angle)

            # command servos to move to target, if not in deadband
            pan_on_target = True
            tilt_on_target = True

            if abs(delta_angle_x) > self.DEADBAND_ANGLE:
                if abs(pan_angle) < self.MAX_PAN:  
                    pub_head_pan.publish(pan_angle)        # Send servo command 
                pan_on_target = False

            if abs(delta_angle_y) > self.DEADBAND_ANGLE:
                if abs(pan_angle) < self.MAX_TILT:    
                    pub_head_tilt.publish(-tilt_angle)     # Send servo command
                tilt_on_target = False

            #if pan_on_target and tilt_on_target:
            #   rospy.loginfo("%s: On target ID %d", self._action_name, person_id)
            #else: 
            #   rospy.loginfo("%s: ID %d: Pan delta = %f, Tilt Delta = %f", 
            #     self._action_name, person_id, delta_angle_x, delta_angle_y) 

            # Max pan/tilt is constrained by system.  Add additional constraints if needed

            self.tracking = True # don't do idle movements

            # SHELDON ONLY
            #side_tilt_angle = 0.0
            #pub_head_sidetilt.publish(side_tilt_angle)


        #self.last_target_time = rospy.Time.now() # reset timer


    #====================================================================
    # Main loop
    def execute_cb(self, goal):

        # Idle Behavior has gone Active!

        # Set servos speed and torque
        SetServoTorque(0.5, head_joints)
        SetServoSpeed(0.35, head_joints) 

        # Move head and arms to ready position
        all_home()

        # Center Camera Head
        pub_head_pan.publish(0.0)
        pub_head_tilt.publish(self.DEFAULT_TILT_ANGLE) # tilt head up to find people more easily
        #pub_head_sidetilt.publish(0.0) # SHELDON ONLY

        if self.enable_random_head_movement:
            rospy.loginfo('%s: random head movements enabled...' % (self._action_name))
        else:
            rospy.loginfo('%s: random head movements DISABLED' % (self._action_name))

        if self.enable_body_tracking:
            rospy.loginfo('%s: waiting for person tracking...' % (self._action_name))
        else:
            rospy.loginfo('%s: body tracking DISABLED' % (self._action_name))

        if self.enable_body_tracking:
            # Enable Subscribers
            #position_sub = rospy.Subscriber("/body_tracker/position", \
            #   BodyTracker, self.position_cb, queue_size=1)
            position_sub = rospy.Subscriber("/body_tracker_array/people", \
                BodyTrackerArray, self.position_cb, queue_size=1)
            
            # pose2d_sub = rospy.Subscriber("/body_tracker/pose2d", Pose2D, self.pose_2d_cb, queue_size=1)
            servo_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_cb) # servos

        while True:

            if self._as.is_preempt_requested():
                break 

            if not self.tracking and self.enable_random_head_movement:   
                # Idle: Move head to constrained random location, at random intervals

                tiltAmt = random.uniform(-0.3, 0.3)
                pub_head_tilt.publish(tiltAmt)


                rospy.loginfo('%s: Doing Random Movement' % (self._action_name))
                panAmt = random.uniform(-0.5, 0.5)
                pub_head_pan.publish(panAmt)

                # SHELDON ONLY
                sidetiltAmt = random.uniform(-0.05, 0.05)
                pub_head_sidetilt.publish(sidetiltAmt)


            self.tracking = False  # do Idle if tracking gets lost

            # delay before next loop
            randSleep = random.randint(10, 35) # tenth seconds
            for i in range(1, randSleep): 
                if self._as.is_preempt_requested():
                    break
                else:
                    time.sleep(0.1)

        # Behavior Exit / Cleanup
        if self.enable_body_tracking:
            position_sub.unregister()
            servo_sub.unregister()

        # Idle always runs until preempted
        rospy.loginfo('%s: Behavior preempted' % self._action_name)
        self._as.set_preempted()

        
if __name__ == '__main__':
    rospy.init_node('idle_behavior')
    server = BehaviorAction(rospy.get_name())
    rospy.spin()
