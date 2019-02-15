#!/usr/bin/env python
# Playback servo positions for scripting of robot behavior
# Implementation note:  
#   This could (should?) be done with arrays instead of individual variables for each servo,
#   but I wanted to be sure I was getting the right servos in each position (for now)


import rospy
import logging
import time
import csv
import math
import sys
import signal

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import UInt16
from std_msgs.msg import UInt32

# for sound effects
import os, thread
#from playsound import playsound
#from pygame import mixer
import pygame


# SHELDON ONLY
#from dynamixel_controllers.srv import TorqueEnable, SetServoTorqueLimit, SetSpeed
from sheldon_servos.servo_joint_list import all_servo_joints, head_joints, right_arm_joints
from sheldon_servos.head_servo_publishers import *
from sheldon_servos.right_arm_servo_publishers import *
from sheldon_servos.left_arm_servo_publishers import *

from sheldon_servos.standard_servo_positions import *
from sheldon_servos.set_servo_speed import *
from sheldon_servos.set_servo_torque import *

# Constants
# Turn State Machine
turn_state_idle = 0
turn_state_initializing = 1
turn_state_turning = 2
turn_state_ending = 3

interrupted = False

SINGLE_STEP_MODE = False  #   <<<<<<<<<<<


def signal_handler(signal, frame):
    global interrupted
    interrupted = True


class PlaybackServoPositions():
    def __init__(self):
        rospy.init_node('playback_servos')

        # Use Python's CSV reader
        #csv_file = open("/home/system/play_servos.csv", "r") 
        csv_file = open("/home/system/script_believer.csv", "r") 
        self.csv_reader = csv.DictReader(csv_file)

        # Publish wheel motor commands as low priority to motor control
        # TODO: Put this in launch file: 
        #   <!-- <remap from="cmd_vel" to="move_base/priority1"/>  --> 
        #self.pub_wheel_motors = rospy.Publisher('~cmd_vel', Twist, queue_size=5)
        self.pub_wheel_motors = rospy.Publisher('move_base/priority1', Twist, queue_size=5)

        # Publish eye color changes
        self.pub_eye_color = rospy.Publisher('/head/eye_color', UInt32, queue_size=2)        

        # Publish robot light control
        self.pub_light_mode = rospy.Publisher('/arm_led_mode', UInt16, queue_size=2)        

        # Publish microphone enable/disable by user (Note: user_enable vs system_enable)
        self.mic_user_enable_pub = rospy.Publisher('microphone/user_enable', Bool, queue_size=1)

        # enable/disable microphone when robot is talking or moving servos.  (Note system_enable vs. user_enable)
        self.mic_system_enable_pub = rospy.Publisher('/microphone/system_enable', Bool, queue_size=1)        

        # Subscribers (examples)
        #compass_sub_ = nh_.subscribe<std_msgs::Float32>("/compass", 10, &WheelControl::compassMsgCallback, this);
        #imu_orientation_sub_ = nh_.subscribe<geometry_msgs::Point32>("/imu_orientation", 10, &WheelControl::imuOrientationMsgCallback, this);

        # subscribe to marker messages by user, to single step for debug
        time_mark_sub = rospy.Subscriber('/user_mark_time', UInt16, self.time_mark_cb) # arbitrary ID assigned by user

        # get path to music file
        self.resource_dir = rospy.get_param('resource_dir', 
          "/home/system/catkin_robot/src/sheldon/sheldon_behaviors/resources/sounds")
        self.music_file = os.path.join(self.resource_dir, "believer_trim2.wav")
        rospy.loginfo("DBG: music file: %s", self.music_file)
        pygame.init()

        rospy.loginfo("playback_servos initialized")


    # ===========================================================================
    # Utility functions

    def cleanup(self):
        # clean everyting up before exiting

        # stop any wheel motion
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        self.pub_wheel_motors.publish(twist)

        # un-mute the microphone
        self.mic_system_enable_pub.publish(True)

        # stop the music if it's still going
        pygame.mixer.music.stop()
        rospy.loginfo("playback_servos Done")

    def time_mark_cb(self, msg):
        rospy.loginfo("got time ID marker from user")
        #id = msg.data
        self.next_step = True    

    def MoveServo(self, servo_name, publisher, value):
        # if servo was written into the file, set it, otherwise just return
        try:
            position = float(value)
            #rospy.loginfo("SET SERVO: Name = %s, Value = %02.4f", servo_name, position )
        except ValueError:
            rospy.loginfo("SET SERVO: Name = %s, NO VALUE for [%s]", servo_name, value )
            return

        if not math.isnan(position):
            # Send command to Servos
            #rospy.loginfo("PLAYBACK MoveServo: Setting servo %s, Value = %02.4f", servo_name, position )
            publisher.publish(position)

    def delay_until_next_step(self, score_time):
        elapsed_time = rospy.Time.now() - self.start_time
        elapsed_sec = elapsed_time.to_sec()
        return (score_time - elapsed_sec)

    #def Turn(self, turn_amount)
        #rospy.loginfo("PLAYBACK Beginning Turn for %3.1f degrees", turn_amount )
        # get starting angle
        #starting_angle = self.current_angle
        # set ending angle    
        # begin turn
        #self.turn_state = turn_state_initializing

    def wheelTurn(self, turn_speed): # positve = left
        # simple turn, does not keep track of distance!
        # uses script timing to control amount (kludge)
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = turn_speed
        self.pub_wheel_motors.publish(twist)

    def wheelMove(self, move_speed):
        # simple move, does not keep track of distance!
        # uses script timing to control amount (kludge)
        twist = Twist()
        twist.linear.x = move_speed; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        self.pub_wheel_motors.publish(twist)


    # ===========================================================================
    # Run Script

    def run(self):

        # Initialize Servo settings
        SetServoTorque(0.5, all_servo_joints)
        SetServoSpeed(0.5, head_joints)
        SetServoSpeed(1.0, right_arm_joints)
        SetServoSpeed(1.0, left_arm_joints)
        SetSingleServoSpeed(1.5, 'right_arm_shoulder_rotate_joint')
        SetSingleServoSpeed(1.5, 'left_arm_shoulder_rotate_joint')

        rospy.loginfo("======================================================")
        rospy.loginfo("               EXECUTING SCRIPT")
        rospy.loginfo("======================================================")

        # Initialize script settings
        self.start_time = rospy.get_rostime()
        self.script_row = 0
        self.turn_state = turn_state_idle
        self.next_step = False

        start_time_offset = 0.0  # how long to wait before first action
        script_score_time = start_time_offset

        # mute the microphone, so the robot does not hear music and servos!
        self.mic_system_enable_pub.publish(False)

        for row in self.csv_reader:

            if interrupted:
                rospy.loginfo("got exit request1, exiting.")
                self.cleanup()
                exit()

            # Read a line from the CSV file
            self.script_row += 1
            # handle script playback timing
            next_step_interval = float(row['time'])
            script_score_time += next_step_interval 

            if SINGLE_STEP_MODE:
                while not self.next_step:
                    if interrupted:
                        rospy.loginfo("got exit request4, exiting.")
                        self.cleanup()
                        exit()

                    #rospy.loginfo("...wait for trigger...")
                    rospy.sleep(0.5)
                self.next_step = False

            else:
                while self.delay_until_next_step(script_score_time) > 1.0:
                    if interrupted:
                        rospy.loginfo("got exit request2, exiting.")
                        self.cleanup()
                        exit()

                    #rospy.loginfo("...sleep big...")
                    rospy.sleep(0.4)
                while self.delay_until_next_step(script_score_time) > 0.05: # tune this for latency
                    #rospy.loginfo("...sleep small...")
                    rospy.sleep(0.025)

            
            # Time to do this action step!
            if interrupted:
                rospy.loginfo("got exit request3, exiting.")
                self.cleanup()
                exit()

            elapsed_time = rospy.Time.now() - self.start_time
            elapsed_sec = elapsed_time.to_sec()
            #rospy.loginfo("PLAYBACK: Executing step %d, time requested = %04.2f, actual = %04.2f",
            #    self.script_row, script_score_time, elapsed_sec )

            field_type = row['type']

            if field_type == 'sound': # TODO, get name from Param1
                #sound_name = row['param1']
                rospy.loginfo("PLAYBACK: =========> Playing Sound")
                #rospy.loginfo("PLAYBACK: =========> Playing Sound [%s]", sound_name)
                # start playing wave file
                pygame.mixer.music.load(self.music_file)
                pygame.mixer.music.play(0)
                #rospy.loginfo("Playing Music" )
                #rospy.sleep(3.0)

            elif field_type == 'marker': # User marker, just display for tuning the script
                marker_number = row['param1']
                rospy.loginfo("PLAYBACK: =========> User Marker [%s]", marker_number)

            elif field_type == 'turn': # Rotate robot at supplied speed/direction
                param1_str = row['param1'] 
                try:
                    rospy.loginfo("PLAYBACK: =========> Wheel Turn [%s]", param1_str)
                    value = float(param1_str)
                    self.wheelTurn(value)
                except ValueError:
                    rospy.logwarn("PLAYBACK: NO PARAM1 for TURN!" )

            elif field_type == 'lights': # Arm (and body?) light setting
                param1_str = row['param1'] 
                try:
                    value = float(param1_str)
                    self.pub_light_mode.publish(value)
                except ValueError:
                    rospy.logwarn("PLAYBACK: NO PARAM1 for lights!" )

            elif field_type == 'eye_color': # Eye Color setting
                param1_str = row['param1'] 
                try:
                    value = float(param1_str)
                    self.pub_eye_color.publish(value)
                except ValueError:
                    rospy.logwarn("PLAYBACK: NO PARAM1 for eye_color!" )

            #elif field_type == 'eye_mode':  #TODO-Optional
            #    eye_color = row['param1']   

            elif field_type == 'speak':       # Say something
                text_to_speak = row['param1'] # param1 contains the phrase to say
                rospy.logwarn("TODO: SPEAKING: %s", text_to_speak )
                # See follow_behavior for example.  Need to handle delay waiting for speech server to start?

            #elif field_type == 'speed': # TODO-Optional
            #    rospy.loginfo("PLAYBACK: Field type: Speed" )

            elif field_type == 'move':


                step = row['step']
                time = row['time']
                comment = row['comment']

                rospy.loginfo("PLAYBACK:  Step: [%s], Time: [%s], Comment: [%s]", step, time, comment)

                # move any servos with valid (non-NAN) values
                self.MoveServo('head_pan',                  pub_head_pan,                   row['head_pan'])
                self.MoveServo('head_tilt',                 pub_head_tilt,                  row['head_tilt'])
                self.MoveServo('head_sidetilt',             pub_head_sidetilt,              row['head_sidetilt'])

                self.MoveServo('right_arm_shoulder_rotate', pub_right_arm_shoulder_rotate,  row['right_arm_shoulder_rotate'])
                self.MoveServo('right_arm_shoulder_lift',   pub_right_arm_shoulder_lift,    row['right_arm_shoulder_lift'])
                self.MoveServo('right_arm_elbow_rotate',    pub_right_arm_elbow_rotate,     row['right_arm_elbow_rotate'])
                self.MoveServo('right_arm_elbow_bend',      pub_right_arm_elbow_bend,       row['right_arm_elbow_bend'])
                self.MoveServo('right_arm_wrist_rotate',    pub_right_arm_wrist_rotate,     row['right_arm_wrist_rotate'])
                self.MoveServo('right_arm_gripper_finger',  pub_right_arm_gripper_finger,   row['right_arm_gripper_finger'])

                self.MoveServo('left_arm_shoulder_rotate',  pub_left_arm_shoulder_rotate,   row['left_arm_shoulder_rotate'])
                self.MoveServo('left_arm_shoulder_lift',    pub_left_arm_shoulder_lift,     row['left_arm_shoulder_lift'])
                self.MoveServo('left_arm_elbow_rotate',     pub_left_arm_elbow_rotate,      row['left_arm_elbow_rotate'])
                self.MoveServo('left_arm_elbow_bend',       pub_left_arm_elbow_bend,        row['left_arm_elbow_bend'])
                self.MoveServo('left_arm_wrist_rotate',     pub_left_arm_wrist_rotate,      row['left_arm_wrist_rotate'])
                self.MoveServo('left_arm_gripper_finger',   pub_left_arm_gripper_finger,    row['left_arm_gripper_finger'])

            elif field_type == '':
                rospy.loginfo("skipping blank line (no field type)")
            elif field_type == '#':
                rospy.loginfo("skipping comment line (field type #)")
            else:
                rospy.logwarn("ERROR! Unknown field_type [%s]", field_type)


        self.cleanup()
        
# ------------------------------------------------------------------
if __name__=='__main__':

    # capture SIGINT signal, e.g., Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)
 
    try:
        playback_servos = PlaybackServoPositions()
        playback_servos.run()

    except rospy.ROSInterruptException:
        pass

    #rospy.spin()


