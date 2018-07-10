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
from sensor_msgs.msg import JointState
import random

from math import radians, degrees
import tf
import os, thread
from playsound import playsound

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

from body_tracker_msgs.msg import BodyTracker
from geometry_msgs.msg import PointStamped, Point, PoseStamped, Pose, Pose2D
#import geometry_msgs.msg

import tf

# TB2S ONLY
#pub_head_pan = rospy.Publisher('/head_pan_controller/command', Float64, queue_size=1)
#pub_head_tilt = rospy.Publisher('/head_tilt_controller/command', Float64, queue_size=1)


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
          '../../resources/sounds/sound_effects')
        rospy.loginfo('%s: PARAM: sound_effects_dir = %s', self._action_name,
            self.sound_effects_dir)

        #self.ding_path = os.path.join(self.sound_effects_dir, "ding.wav")
        #rospy.loginfo("DBG: DING PATH: %s", self.ding_path)
        #playsound(self.ding_path) # test sound

        #====================================================================


        self.tracking = False
        self.joint_state = JointState() # for reading servo positions
        #self.astra_target = list()

        # Remember which person to track (so the camera does not oscilate)
        self.id_to_track = 0  # 0 = not tracking anyone
        self.last_target_time = rospy.Time.now() # start timer

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


    def gesture_cb(self, msg):
        rospy.loginfo('%s: ERROR ERROR got gesture_cb message' % (self._action_name))
        return

        gesture = msg.x # position in radians from center of camera lens
        # msg.y not used
        person_id = int(msg.theta)

        # whenever we get a gesture, force this to be the current user
        # TODO decode the gestures to what kind they are...
        self.id_to_track = person_id
        self.last_target_time = rospy.Time.now() # reset timer


    #====================================================================
    # 3D Pose Tracking:  Message contains xyz of person 
    #                    position is relative to the robot  
    def body_pose_cb(self, msg):

        rospy.loginfo('%s: ERROR!  ERROR!  got body_pose message' % (self._action_name))
        return # THIS CB IS DISABLED

        # position component of the target pose stored as a PointStamped() message.

        # create a PointStamped structure to transform via transformPoint
        target = PointStamped()
        target.header.frame_id = msg.header.frame_id
        target.point = msg.pose.position

        if target.point.z == 0.0:
            rospy.loginfo('%s: skipping blank message' % (self._action_name))
            return
        # frame should be "camera_depth_frame"
        #target = self.tf.transformPoint(self.camera_link, raw_target)           

        rospy.loginfo("%s: Body Tracker: Tracking person at %f, %f, %f", self._action_name,
            target.point.x, target.point.y, target.point.z)

        # convert from xyz to pan tilt angles
        # TODO: 1) Handle Children - currently assumes everyone is 6 foot tall!
        #       2) What happens if robot bows? 
        if target.point.x < 0.2:     # min range of most depth cameras
            #rospy.loginfo("%s: Body Tracker: Bad Distance (x) value! %f", 
            #  self._action_name, target.point.x)
            return

        # math shortcut for approx radians
        pan_angle =  target.point.y / target.point.x

        # OPTION 1: Track actual target height 
        #person_head_offset_radians = 0.52   # TB2S value - TODO Tune this 
        #tilt_angle = (target.point.z / target.point.x) + person_head_offset_radians 

        # OPTION 2: Guess height, based upon distance to person
        # FUTURE: combine the two, use "guess" when person is too close?
        tilt_angle = 0.4 / target.point.x # SHELDON, CHEST MOUNTED camera

        rospy.loginfo("%s: Body Tracker: Pan = %f (%f), Tilt = %f (%f)", self._action_name, 
            pan_angle, degrees(pan_angle), tilt_angle, degrees(tilt_angle))

        # Send servo commands
        if abs(pan_angle) > MAX_PAN:    # just over 45 degrees - TODO put in actual limits here!
            rospy.loginfo("%s: Body Tracker: Pan %f exceeds MAX", self._action_name, pan_angle)
            return
        if abs(tilt_angle) > MAX_TILT:    # Limit vertical to assure good tracking
            rospy.loginfo("%s: Body Tracker: Tilt %f exceeds MAX", self._action_name, tilt_angle)
            return

        pub_head_pan.publish(pan_angle)
        pub_head_tilt.publish(-tilt_angle)

        # SHELDON ONLY
        #sidetiltAmt = 0.0
        #pub_head_sidetilt.publish(sidetiltAmt)

        self.tracking = True # don't do idle movements


    #====================================================================
    # 2D Tracking:  Message contains person horizontal (x) and vertical (y)
    #               position is relative to the depth image.  
    def position_cb(self, msg):
        #rospy.loginfo('%s: got position_cb message' % (self._action_name))
         
        delta_angle_x = msg.position2d.x # position in radians from center of camera lens
        delta_angle_y = msg.position2d.y  
        person_id = msg.body_id 
        gesture = msg.gesture


        if self.id_to_track == 0:
            self.id_to_track = person_id # no id assigned yet, so use this one
            rospy.loginfo("%s: Tracking Person_ID %d", self._action_name, person_id)

        elif gesture > -1:
            self.id_to_track = person_id # got a gesture, so use this ID
            #playsound(self.ding_path) # indicate gesture recognized with a sound
            rospy.loginfo("%s: ---------------------> Person_ID %d Gesture detected: %d", 
                self._action_name, person_id, gesture)
            rospy.loginfo("%s: Tracking Person_ID %d", self._action_name, person_id)

        elif person_id != self.id_to_track:
            # not the right person, see if the old one timed out

            time_since_last_target = rospy.Time.now() - self.last_target_time
            if time_since_last_target > rospy.Duration.from_sec(3.0): 
                # target timed out, use this one  TODO - see which target is closest?
                rospy.loginfo("%s: Body Tracker: ID %d Timed out, changing to ID %d", 
                    self._action_name, self.id_to_track, person_id )
                self.id_to_track = person_id
            else:            
                rospy.loginfo("%s: Body Tracker: Tracking ID %d, so skipping pose2D for ID %d", 
                    self._action_name, self.id_to_track, person_id )
                return

        self.last_target_time = rospy.Time.now() # reset timer


        # Calculate amount to move
        #rospy.loginfo("%s: Person %d 2D Delta:  x = %f,  y = %f", 
        #   self._action_name, person_id, delta_angle_x, delta_angle_y )

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
                pub_head_pan.publish(pan_angle)
            pan_on_target = False

        if abs(delta_angle_y) > self.DEADBAND_ANGLE:
            if abs(pan_angle) < self.MAX_TILT:    
                pub_head_tilt.publish(-tilt_angle)
            tilt_on_target = False

        #if pan_on_target and tilt_on_target:
        #   rospy.loginfo("%s: On target ID %d", self._action_name, person_id)
        #else: 
        #   rospy.loginfo("%s: ID %d: Pan delta = %f, Tilt Delta = %f", 
        #     self._action_name, person_id, delta_angle_x, delta_angle_y) 


        # SHELDON ONLY
        #side_tilt_angle = 0.0
        #pub_head_sidetilt.publish(side_tilt_angle)

        self.tracking = True # don't do idle movements

        # Send servo commands
        if abs(pan_angle) > 1.0:    # just over 45 degrees - TODO put in actual limits here!
            rospy.loginfo("%s: Body Tracker: Pan %f exceeds MAX", self._action_name, pan_angle)
            return
        if abs(tilt_angle) > 1.0:  # just over 45 degrees
            rospy.loginfo("%s: Body Tracker: Tilt %f exceeds MAX", self._action_name, tilt_angle)
            return


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
            #rospy.Subscriber("/body_tracker/pose", PoseStamped, self.body_pose_cb, queue_size=1)
            position_sub = rospy.Subscriber("/body_tracker/position", BodyTracker, self.position_cb, queue_size=1)
            # pose2d_sub = rospy.Subscriber("/body_tracker/pose2d", Pose2D, self.pose_2d_cb, queue_size=1)
            servo_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_cb) # servos
            #gesture_sub = rospy.Subscriber('/body_tracker/gesture', Pose2D, self.gesture_cb)

        while True:

            if self._as.is_preempt_requested():
                break 

            if not self.tracking and self.enable_random_head_movement:   
                # Idle: Move head to constrained random location, at random intervals

                tiltAmt = random.uniform(-0.3, 0.3)
                pub_head_tilt.publish(tiltAmt)


                # rospy.loginfo('%s: Doing Random Movement' % (self._action_name))
                panAmt = random.uniform(-0.5, 0.5)
                pub_head_pan.publish(panAmt)

                # SHELDON ONLY
                #sidetiltAmt = 0.0   # TODO random.uniform(-0.05, 0.05)
                #pub_head_sidetilt.publish(sidetiltAmt)


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
            #pose2d_sub.unregister()
            servo_sub.unregister()
            #gesture_sub.unregister()

        # Idle always runs until preempted
        rospy.loginfo('%s: Behavior preempted' % self._action_name)
        self._as.set_preempted()

        
if __name__ == '__main__':
    rospy.init_node('idle_behavior')
    server = BehaviorAction(rospy.get_name())
    rospy.spin()
