#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
import time


# ================================================================
# UTILITIES

def head_center():
    print("-----> head_center")
    pub_sidetilt.publish(0.0)
    pub_tilt.publish(0.0)
    pub_pan.publish(0.0)

def head_down():
    print("-----> head_down")
    pub_sidetilt.publish(0.0)
    pub_tilt.publish(1.26)
    pub_pan.publish(0.0)


# RIGHT ARM

def right_gripper_open():
    print("-----> right_gripper_open")
    pub_right_arm_gripper.publish(0.0)

def right_gripper_close():
    print("-----> right_gripper_close")
    pub_right_arm_gripper.publish(-2.0)

def right_arm_down():
    print("-----> right_arm_down")
    pub_right_arm_elbow_bend.publish(0.0)
    pub_right_arm_elbow_rotate.publish(0.0)
    pub_right_arm_wrist_rotate.publish(0.0)
    #pub_right_arm_lift.publish(0.25) # keep away from body
    pub_right_arm_shoulder.publish(-0.4)
    right_gripper_open()

def right_arm_home():
    print("-----> right_arm_home")
    #pub_right_arm_lift.publish(0.25)
    pub_right_arm_elbow_rotate.publish(0.0)
    pub_right_arm_elbow_bend.publish(2.0)
    pub_right_arm_wrist_rotate.publish(0.0)
    pub_right_arm_gripper.publish(0.25)
    pub_right_arm_shoulder.publish(-1.0)

def right_arm_extend():
    print("-----> right_arm_extend")
    pub_sidetilt.publish(0.0)
    pub_tilt.publish(0.25)
    pub_pan.publish(-0.3)
    #pub_right_arm_lift.publish(0.25)
    pub_right_arm_elbow_rotate.publish(-0.25)
    pub_right_arm_elbow_bend.publish(1.8)
    pub_right_arm_wrist_rotate.publish(0.0)
    #pub_right_arm_gripper.publish(-0.8)
    pub_right_arm_shoulder.publish(1.0)

def right_arm_extend_full(): 
    print("-----> right_arm_extend_full")
    pub_right_arm_lift.publish(0.25)
    pub_right_arm_elbow_rotate.publish(0.0)
    pub_right_arm_elbow_bend.publish(0.0)
    pub_right_arm_wrist_rotate.publish(0.0)
    #pub_right_arm_gripper.publish(0.25)
    pub_right_arm_shoulder.publish(3.0)




# LEFT ARM
def left_gripper_open():
    print("-----> left_gripper_open")
    pub_left_arm_gripper.publish(0.0)

def left_gripper_close():
    print("-----> left_gripper_close")
    pub_left_arm_gripper.publish(1.5)

def left_arm_down():
    print("-----> left_arm_down")
    pub_left_arm_elbow_bend.publish(0.0)
    pub_left_arm_elbow_rotate.publish(0.0)
    pub_left_arm_wrist_rotate.publish(0.0)
    pub_left_arm_gripper.publish(0.0)
    pub_left_arm_lift.publish(0.25) # keep away from body
    pub_left_arm_shoulder.publish(-0.4)

def left_arm_home():
    print("-----> left_arm_home")
    pub_left_arm_lift.publish(0.25)
    pub_left_arm_elbow_rotate.publish(0.0)
    pub_left_arm_elbow_bend.publish(2.0)
    pub_left_arm_wrist_rotate.publish(0.0)
    # pub_left_arm_gripper.publish(0.25)
    pub_left_arm_shoulder.publish(-1.0)

def left_arm_extend():    
    print("-----> left_arm_extend")
    pub_sidetilt.publish(0.0)
    pub_tilt.publish(0.25)
    pub_pan.publish(0.3)
    pub_left_arm_lift.publish(0.25)
    pub_left_arm_elbow_rotate.publish(0.3)
    pub_left_arm_elbow_bend.publish(1.8)
    pub_left_arm_wrist_rotate.publish(0.0)
    #pub_left_arm_gripper.publish(0.8)
    pub_left_arm_shoulder.publish(1.0)

def left_arm_extend_full():
    print("-----> left_arm_extend_full")
    pub_left_arm_lift.publish(0.25)
    pub_left_arm_elbow_rotate.publish(0.0)
    pub_left_arm_elbow_bend.publish(0.0)
    pub_left_arm_wrist_rotate.publish(0.0)
    #pub_left_arm_gripper.publish(0.25)
    pub_left_arm_shoulder.publish(3.0)

def left_arm_wave():
    print("-----> left_arm_wave")
    pub_sidetilt.publish(0.0)
    pub_tilt.publish(-0.1)
    pub_pan.publish(0.0)
    pub_left_arm_lift.publish(0.25)
    pub_left_arm_elbow_rotate.publish(0.0)
    pub_left_arm_elbow_bend.publish(2.2)
    pub_left_arm_wrist_rotate.publish(0.0)
    #pub_left_arm_gripper.publish(0.8)
    pub_left_arm_shoulder.publish(1.7)
    time.sleep(2.5)

    for i in range(0,2):
        pub_left_arm_elbow_rotate.publish(0.20)
        time.sleep(0.5)
        pub_left_arm_elbow_rotate.publish(-0.20)
        time.sleep(0.5)

    left_arm_home()
    head_center()


# ================================================================


def callback(data):
    if data.buttons[5] == 1: # Right Top Trigger
    	pub_pan.publish(data.axes[2] * -1.0)
    	pub_tilt.publish(data.axes[3] * -1.0)
    	pub_sidetilt.publish(data.axes[0] * -1.0)
        #if data.axes[4] > 0.0:
        #    pub_sidetilt.publish(0.25)
	#elif data.axes[4] < 0.0:
    	#    pub_sidetilt.publish(-0.25)
        #else:
    	#    pub_sidetilt.publish(0.0)

    if data.buttons[6] == 1:  # Left Bottom Trigger USED BY WHEELS!
    	print("Left Bottom Trigger USED BY WHEELS!!")

    if data.buttons[7] == 1:  # Right Bottom Trigger
    	print("Right Bottom Trigger")
 

# BUTTONS DAVES
    if data.buttons[1] == 1: # Green A button - Ready position
        print("Red B button - Sleep")
        head_center()
        right_arm_home()
        left_arm_home()

    elif data.buttons[2] == 1: # Red B button - Sleep position (down)
        print("Green A button - Ready")
        head_down()
        right_arm_down()
        left_arm_down()

    elif data.buttons[0] == 1: # Blue X button
        print("Blue X button")
        left_arm_wave()

    elif data.buttons[3] == 1: # Yellow Y button
        print("Yellow Y button")


    
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/joy", Joy, callback)
    rospy.spin()

if __name__ == '__main__':
    pub_pan = rospy.Publisher('/head_pan_joint/command', Float64, queue_size=1)
    pub_tilt = rospy.Publisher('/head_tilt_joint/command', Float64, queue_size=1)
    pub_sidetilt = rospy.Publisher('/head_sidetilt_joint/command', Float64, queue_size=1)

    pub_left_arm_lift = rospy.Publisher('/left_arm_lift_joint/command', Float64, queue_size=1)
    pub_left_arm_elbow_rotate = rospy.Publisher('/left_arm_elbow_rotate_joint/command', Float64, queue_size=1)
    pub_left_arm_elbow_bend = rospy.Publisher('/left_arm_elbow_bend_joint/command', Float64, queue_size=1)
    pub_left_arm_wrist_rotate = rospy.Publisher('/left_arm_wrist_rotate_joint/command', Float64, queue_size=1)
    pub_left_arm_gripper = rospy.Publisher('/left_arm_gripper_joint/command', Float64, queue_size=1)
    pub_left_arm_shoulder = rospy.Publisher('/left_arm_shoulder_controller/command', Float64, queue_size=1)

    pub_right_arm_lift = rospy.Publisher('/right_arm_lift_joint/command', Float64, queue_size=1)
    pub_right_arm_elbow_rotate = rospy.Publisher('/right_arm_elbow_rotate_joint/command', Float64, queue_size=1)
    pub_right_arm_elbow_bend = rospy.Publisher('/right_arm_elbow_bend_joint/command', Float64, queue_size=1)
    pub_right_arm_wrist_rotate = rospy.Publisher('/right_arm_wrist_rotate_joint/command', Float64, queue_size=1)
    pub_right_arm_gripper = rospy.Publisher('/right_arm_gripper_joint/command', Float64, queue_size=1)
    pub_right_arm_shoulder = rospy.Publisher('/right_arm_shoulder_controller/command', Float64, queue_size=1)



    listener()
