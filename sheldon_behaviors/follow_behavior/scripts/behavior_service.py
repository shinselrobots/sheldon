#! /usr/bin/env python

import rospy
import actionlib
import behavior_common.msg
import time
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from math import radians, degrees
import tf
import os, thread
from playsound import playsound

# for talking
# import actionlib
import actionlib.action_client
import audio_and_speech_common.msg

from geometry_msgs.msg import PointStamped, Point, PoseStamped, Pose, Pose2D
from body_tracker_msgs.msg import BodyTracker
from enum import Enum
import tf

# SHELDON ONLY
#from dynamixel_controllers.srv import TorqueEnable, SetServoTorqueLimit, SetSpeed
from sheldon_servos.servo_joint_list import all_joints, head_joints, right_arm_joints
from sheldon_servos.head_servo_publishers import *

from sheldon_servos.standard_servo_positions import *
from sheldon_servos.set_servo_speed import *
from sheldon_servos.set_servo_torque import *

#from tb2s_pantilt.set_servo_speed import *

# TB2S ONLY
#pub_head_pan = rospy.Publisher('/head_pan_controller/command', Float64, queue_size=1)
#pub_head_tilt = rospy.Publisher('/head_tilt_controller/command', Float64, queue_size=1)
#pub_move = rospy.Publisher('/cmd_vel_mux/behavior', Twist, queue_size=5)
pub_move = rospy.Publisher('move_base/priority1', Twist, queue_size=5)  # Low Priority

class BehaviorAction(object):

    def __init__(self, name):

        self._action_name = name
        rospy.loginfo('%s: Initializing Python behavior service' % (self._action_name))
        self._as = actionlib.SimpleActionServer(self._action_name, 
          behavior_common.msg.behaviorAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

        # Behavior Settings
        # TODO - How to get these parameters passed from launch file to here?
        # for now, just set here.
        self.enable_body_tracking = True  # rospy.get_param('~enable_body_tracking', True)
        # rospy.loginfo('%s: PARAM: enable_body_tracking = %d'.(self._action_name), 
        #   self.enable_body_tracking)
        self.enable_random_head_movement = False  # rospy.get_param('~enable_random_head_movement', True)
        # rospy.loginfo('%s: PARAM: enable_random_head_movement = %d'.(self._action_name),
        #   self.enable_random_head_movement)
        self.camera_link = 'camera_link' # rospy.get_param('~camera_link', 'camera_link')
        self.head_pan_joint = rospy.get_param('~head_pan_joint', 'head_pan_joint')
        self.head_tilt_joint = rospy.get_param('~head_tilt_joint', 'head_tilt_joint')

        self.resource_dir = rospy.get_param('resource_dir', 
          "/home/system/catkin_robot/src/sheldon/sheldon_behaviors/follow_behavior/scripts/resources")
        self.ding_path = os.path.join(self.resource_dir, "ding.wav")
        rospy.loginfo("DBG: DING PATH: %s", self.ding_path)

        # constants
        self.MAX_PAN = 1.5708 #  90 degrees
        self.MAX_TILT = 0.60  #  Limit vertical to assure good tracking
        self.DEADBAND_ANGLE = 0.0872665 # 5 deg deadband in middle to prevent osc

        self.FIRST_TARGET_TIMEOUT_SECONDS = 5.0 
        self.TRACKING_TIMEOUT_SECONDS = 3.0
        self.GESTURE_TIMEOUT_SECONDS = 10.0
        self.DEFAULT_TILT_ANGLE = 0.0  # TB2S = -0.35 # tilt head up slightly to find people more easily

        # Timeout timers
        self.first_target_timer = 0
        self.tracking_timer = 0
        self.gesture_timer = 0

        self.joint_state = JointState() # for reading servo positions
        #self.astra_target = list()
        self.id_to_track = 0  # 0 = not tracking anyone

        # Initialize State Machine
        self.TrackingState = Enum('TrackingState', 
          'WAITING_FOR_FIRST_ID TRACKING WAITING_FOR_GESTURE')
        self.tracking_state = self.TrackingState.WAITING_FOR_FIRST_ID

        rospy.loginfo("%s: init complete", self._action_name)
        #playsound(self.ding_path)


    #------------------------------------------------------------------------
    def MoveRobot(self, tracking_angle, tracking_distance):

        rospy.loginfo("%s: MoveRobot: Angle = %f,  Distance = %f", 
            self._action_name, tracking_angle, tracking_distance)

        TURN_MULTIPLIER = 3.0
        FORWARD_ACCELERATION_MULTIPLIER = 0.15
        TURN_DEADBAND = 0.01
        FOWARD_DEADBAND = 1.4
        BACKWARD_DEADBAND = 1.1
        BACKWARD_ACCELERATION_MULTIPLIER = 0.1
        MIN_SPEED = 0.2
        MAX_TURN = 0.3
        speed = 0.0
        turn = 0.0
 
        # Set Turn.  Limit to reasonable values in case body tracker gets confused
        turn = tracking_angle * TURN_MULTIPLIER;
        if turn > MAX_TURN:
            turn = MAX_TURN
        elif turn < -MAX_TURN:
            turn = -MAX_TURN

        # Set Speed
        if tracking_distance > FOWARD_DEADBAND:
            speed = FORWARD_ACCELERATION_MULTIPLIER * (tracking_distance * tracking_distance);
            if speed < MIN_SPEED:
                speed = MIN_SPEED
            rospy.loginfo("%s: MoveRobot: forward speed = %f", self._action_name, speed)

        elif (tracking_distance < BACKWARD_DEADBAND) and tracking_distance > 0:
            speed = -BACKWARD_ACCELERATION_MULTIPLIER * (1 / 
              (tracking_distance * tracking_distance));
            if speed > -MIN_SPEED:
                speed = -MIN_SPEED
            rospy.loginfo("%s: MoveRobot: backup speed = %f", self._action_name, speed)

        # Publish Move Command
        rospy.loginfo("%s: MoveRobot: Pub Twist: speed = %f,  turn = %f", 
            self._action_name, speed, turn)
        twist = Twist()
        twist.linear.x = speed; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = turn
        pub_move.publish(twist)


    def StopRobot(self):
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub_move.publish(twist)

    #------------------------------------------------------------------------
    # Callbacks

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


    #------------------------------------------------------------------------
    # 3D Pose Tracking:  Message contains xyz of person 
    #                    position is relative to the robot
    # we use this to actually follow the person!

    def body_pose_cb(self, msg):
        rospy.loginfo('%s: ERROR ERROR got 3D body_pose message' % (self._action_name))
        return

        if person_id != self.id_to_track:
            # not the right person, skip
            rospy.loginfo("%s: Body Tracker: Tracking ID is %d, skipping pose2D for ID %d", 
                self._action_name, self.id_to_track, person_id )
            return

       # position component of the target pose stored as a PointStamped() message.

        # create a PointStamped structure to transform via transformPoint
        target = PointStamped()
        target.header.frame_id = msg.header.frame_id
        target.point = msg.pose.position

        if target.point.z == 0.0:
            rospy.loginfo('%s: skipping blank message' % (self._action_name))
            return

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
        sidetiltAmt = 0.0
        pub_head_sidetilt.publish(sidetiltAmt)


    #------------------------------------------------------------------------
    # 2D Tracking:  Message contains person horizontal (x) and vertical (y)
    #               position is relative to the depth image.  
    # we use this to control the head tracking (less oscillation?)

    def position_cb(self, msg):
        #rospy.loginfo('%s: got position_cb message' % (self._action_name))

        delta_angle_x = msg.position2d.x # position in radians from center of camera lens
        delta_angle_y = msg.position2d.y
        person_tracking_distance = msg.position2d.z
        person_id = msg.body_id 
        gesture = msg.gesture

        if self.tracking_state == self.TrackingState.WAITING_FOR_FIRST_ID:
            # this is the first tracking frame we have received
            self.id_to_track = person_id # no id assigned yet, so use this one
            self.tracking_timer = rospy.Time.now() # start tracking timer
            self.tracking_state = self.TrackingState.TRACKING
            rospy.loginfo("%s: Tracking Person_ID %d", self._action_name, person_id)

        elif self.tracking_state == self.TrackingState.WAITING_FOR_GESTURE:
            # lost person, waiting to restart tracking with a gesture
            if gesture > -1:
                playsound(self.ding_path)
                self.id_to_track = person_id # got a gesture, so use this ID
                self.tracking_timer = rospy.Time.now() # start tracking timer
                self.tracking_state = self.TrackingState.TRACKING
                rospy.loginfo("%s: ---------------------> Person_ID %d Gesture detected: %d", 
                    self._action_name, person_id, gesture)
                rospy.loginfo("%s: Tracking Person_ID %d", self._action_name, person_id)

                # say something
                rospy.loginfo("Talking")
                goal = audio_and_speech_common.msg.speechGoal(
                    text_to_speak="ok, i see you, lets go")
                self.speech_client.send_goal(goal)
                #result = self.speech_client.wait_for_result()   # wait for speech to complete
                #rospy.loginfo("Speech goal returned result: %d", result)
            else:
                return # continue waiting

        elif self.tracking_state != self.TrackingState.TRACKING:
            rospy.logwarn("%s: TRACKING STATE ERROR, unknown state: %d", 
                self._action_name, self.tracking_state) 
            return

        if person_id != self.id_to_track:
            # not the right person, skip
            time_since_last_target = rospy.Time.now() - self.tracking_timer
            rospy.loginfo("%s: Skipping ID %d, tracking ID is %d, timer is %f", 
                self._action_name, self.id_to_track, person_id, time_since_last_target.to_sec() )
            return

        # --------------------------------------------------------------------
        # GOT A PERSON TO TRACK
        rospy.loginfo("%s: Body Tracker: Person %d 2D Delta:  x = %f,  y = %f", 
            self._action_name, person_id, delta_angle_x, delta_angle_y )

        # Get the current servo pan and tilt position
        try:
            current_pan = self.joint_state.position[
              self.joint_state.name.index(self.head_pan_joint)]
            current_tilt = self.joint_state.position[
              self.joint_state.name.index(self.head_tilt_joint)] * -1.0
        except:
            return

        #rospy.loginfo("%s: Body Tracker: Current Servo:  Pan = %f,  Tilt = %f", 
        #    self._action_name, current_pan, current_tilt)

        # add target position to current servo position
        pan_angle  = current_pan  + (delta_angle_x * 0.75) #shoot for less
        tilt_angle = current_tilt + (delta_angle_y * 0.75)
        #rospy.loginfo("%s: Body Tracker: Servo Command:  Pan = %f,  Tilt = %f", 
        #    self._action_name, pan_angle, tilt_angle)
        person_tracking_angle = pan_angle

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
        #    rospy.loginfo("%s: Body Track On target ID %d", 
        #      self._action_name, person_id)
        #else: 
        #    rospy.loginfo("%s: Body Track ID %d: Pan delta = %f, Tilt Delta = %f", 
        #      self._action_name, person_id, delta_angle_x, delta_angle_y) 

        # SHELDON ONLY
        #side_tilt_angle = 0.0
        #pub_head_sidetilt.publish(side_tilt_angle)

        # Move the robot to follow person
        self.MoveRobot(person_tracking_angle, person_tracking_distance)

        self.tracking_timer = rospy.Time.now() # reset tracking timer



    #------------------------------------------------------------------------
    # Execute Behavior - Starts when this behavior goes active
    def execute_cb(self, goal):
        #r = rospy.Rate(1)

        # Initalize state engine
        self.tracking_state = self.TrackingState.WAITING_FOR_FIRST_ID

        # initialize Speech
        rospy.loginfo("Waiting for speech server (press ctrl-c to cancel at anytime)")
        self.speech_client = actionlib.SimpleActionClient("/speech_service", 
            audio_and_speech_common.msg.speechAction)
        self.speech_client.wait_for_server()

        # Subscribers - begin listening for tracking messages
        if self.enable_body_tracking:
            position_sub = rospy.Subscriber("/body_tracker/position", BodyTracker, self.position_cb, queue_size=1)
            #rospy.Subscriber("/body_tracker/pose", PoseStamped, self.body_pose_cb, queue_size=1)
            #pose2d_sub = rospy.Subscriber("/body_tracker/pose2d", Pose2D, self.pose_2d_cb, queue_size=1)
            servo_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_cb) # for servos
            #gesture_sub = rospy.Subscriber('/body_tracker/gesture', Pose2D, self.gesture_cb)

        # Set servos speed and torque
        # TODO SetServoTorque(0.5, head_joints)
        SetServoSpeed(0.35, head_joints)

        # Center Camera Head
        #pub_head_pan.publish(0.0)
        #pub_head_tilt.publish(self.DEFAULT_TILT_ANGLE) # tilt head up to find people more easily
        #pub_head_sidetilt.publish(0.0) # SHELDON ONLY

        # start with robot stopped
        self.StopRobot()

        # say something
        rospy.loginfo("Talking")
        goal = audio_and_speech_common.msg.speechGoal(text_to_speak="ok, i will follow you")
        self.speech_client.send_goal(goal)
        #result = self.speech_client.wait_for_result()   # wait for speech to complete
        #rospy.loginfo("Speech goal returned result: %d", result)

        # Initialize timers
        self.first_target_timer = rospy.Time.now()
        self.tracking_timer = rospy.Time.now()
        self.gesture_timer = rospy.Time.now()

        rospy.loginfo('==================================================')
        rospy.loginfo('==================================================')
        rospy.loginfo('==================================================')
        rospy.loginfo('%s: waiting to spot first person...' % (self._action_name))

        # -------- LOOP --------
        while True: 
            rospy.loginfo('==================================================')
            rospy.loginfo('%s: Tracking State: %s', self._action_name, self.tracking_state.name)
            rospy.loginfo('==================================================')

            if self._as.is_preempt_requested():
                break # higher priority behavior requested

            if self.tracking_state == self.TrackingState.WAITING_FOR_FIRST_ID:
                time_waiting_for_first_target = rospy.Time.now() - self.first_target_timer
                if time_waiting_for_first_target > rospy.Duration.from_sec(
                    self.FIRST_TARGET_TIMEOUT_SECONDS): 
                    rospy.logwarn("%s: time_waiting_for_first_target: I DONT SEE ANYONE TO TRACK!", 
                        self._action_name)
                    # say something
                    rospy.loginfo("Talking")
                    goal = audio_and_speech_common.msg.speechGoal(text_to_speak="darn, I dont see anyone to follow")
                    self.speech_client.send_goal(goal)
                    #result = self.speech_client.wait_for_result()   # wait for speech to complete
                    #rospy.loginfo("Speech goal returned result: %d", result)
                    break # did not find a person to track

            elif self.tracking_state == self.TrackingState.TRACKING:
                time_since_last_target = rospy.Time.now() - self.tracking_timer
                rospy.loginfo("%s: State = TRACKING.  Time since last frame: %d", 
                    self._action_name, time_since_last_target.to_sec() )
                if time_since_last_target > rospy.Duration.from_sec(
                    self.TRACKING_TIMEOUT_SECONDS): 
                    # target timed out!  Lost User!
                    pub_head_tilt.publish(self.DEFAULT_TILT_ANGLE) # Set tilt for optimal capture
                    rospy.loginfo("%s: LOST USER! waiting for gesture...", self._action_name)
                    self.gesture_timer = rospy.Time.now()
                    self.tracking_state = self.TrackingState.WAITING_FOR_GESTURE
                    # say something
                    rospy.loginfo("Talking")
                    goal = audio_and_speech_common.msg.speechGoal(
                        text_to_speak="darn, I lost you.  please wave or something")
                    self.speech_client.send_goal(goal)
                    #result = self.speech_client.wait_for_result()   # wait for speech to complete
                    #rospy.loginfo("Speech goal returned result: %d", result)

            elif self.tracking_state == self.TrackingState.WAITING_FOR_GESTURE:
                time_waiting_for_gesture = rospy.Time.now() - self.gesture_timer
                if time_waiting_for_gesture > rospy.Duration.from_sec(
                    self.GESTURE_TIMEOUT_SECONDS): 
                    rospy.logwarn("%s: time_waiting_for_gesture: I DONT SEE ANY GESTURES!", 
                        self._action_name)
                    # say something
                    rospy.loginfo("Talking")
                    goal = audio_and_speech_common.msg.speechGoal(
                        text_to_speak="i have stopped following, now what")
                    self.speech_client.send_goal(goal)
                    #result = self.speech_client.wait_for_result()   # wait for speech to complete
                    #rospy.loginfo("Speech goal returned result: %d", result)
                    break # did not find a gesture to restart tracking

            else:
                rospy.logwarn("%s: BAD STATE!", self._action_name )

            time.sleep(0.5)

    
        #----------------------------------------------------------------
        # Behavior Exit

        # Stop wheels before exiting
        self.StopRobot()

        rospy.loginfo('%s: Behavior Exiting', self._action_name)
        position_sub.unregister()
        #pose2d_sub.unregister()
        servo_sub.unregister()
        #gesture_sub.unregister()

        # Report exit status
        if self._as.is_preempt_requested():
            self._as.set_preempted();
        else:
            self._as.set_succeeded();
 
       
if __name__ == '__main__':
    rospy.init_node('follow_behavior')
    server = BehaviorAction(rospy.get_name())
    rospy.spin()
