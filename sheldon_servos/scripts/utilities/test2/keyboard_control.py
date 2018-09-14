#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Float64

# 90 degrees = 1.57, 45 = 0.785

# global
g_left_draw_lift_origin = 0.70 # 0.57

# ================================================================
# HEAD

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

# ================================================================
# RIGHT ARM

def right_gripper_open():
    print("-----> right_gripper_open")
    pub_right_arm_gripper_finger.publish(0.0)

def right_gripper_open_half():
    print("-----> right_gripper_open")
    pub_right_arm_gripper_finger.publish(-1.0)

def right_gripper_close():
    print("-----> right_gripper_close")
    pub_right_arm_gripper_finger.publish(-2.0)

def right_get_item():
    pub_right_arm_gripper_finger.publish(-0.8)
    right_arm_extend()

def right_arm_down():
    print("-----> right_arm_down")
    pub_right_arm_elbow_bend.publish(0.0)
    pub_right_arm_elbow_rotate.publish(0.0)
    pub_right_arm_wrist_rotate.publish(0.0)
    #pub_right_arm_lift.publish(0.25) # keep away from body
    pub_right_arm_shoulder.publish(-0.4)
    right_gripper_open_half()

def right_arm_home():
    print("-----> right_arm_home")
    #pub_right_arm_lift.publish(0.25)
    pub_right_arm_elbow_rotate.publish(0.0)
    pub_right_arm_elbow_bend.publish(2.0)
    pub_right_arm_wrist_rotate.publish(0.0)
    #pub_right_arm_gripper_finger.publish(0.25)
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
    #pub_right_arm_gripper_finger.publish(-0.8)
    pub_right_arm_shoulder.publish(1.0)

def right_arm_extend_full(): 
    print("-----> right_arm_extend_full")
    pub_right_arm_lift.publish(0.25)
    pub_right_arm_elbow_rotate.publish(0.0)
    pub_right_arm_elbow_bend.publish(0.0)
    pub_right_arm_wrist_rotate.publish(0.0)
    #pub_right_arm_gripper_finger.publish(0.25)
    pub_right_arm_shoulder.publish(3.0)

def right_arm_write_ready():
    print("-----> right_arm_write_ready")
    #pub_right_arm_lift.publish(0.57)
    pub_right_arm_elbow_rotate.publish(-1.57)
    pub_right_arm_elbow_bend.publish(0.57)
    pub_right_arm_wrist_rotate.publish(1.57)
    #pub_right_arm_gripper_finger.publish(0.25)
    pub_right_arm_shoulder.publish(2.8)





# ================================================================
# LEFT ARM
def left_gripper_open():
    print("-----> left_gripper_open")
    pub_left_arm_gripper_finger.publish(0.0)

def left_gripper_open_half():
    print("-----> left_gripper_open")
    pub_left_arm_gripper_finger.publish(0.8)

def left_gripper_close():
    print("-----> left_gripper_close")
    pub_left_arm_gripper_finger.publish(1.5)

def left_get_item():
    pub_right_arm_gripper_finger.publish(0.8)
    right_arm_extend()

def left_pen_up():
    print("-----> left_pen_up")
    pub_left_arm_elbow_rotate.publish(1.57 - 0.10)
    time.sleep(1.0)

def left_pen_down():
    print("-----> left_pen_down")
    pub_left_arm_elbow_rotate.publish(1.57)
    time.sleep(1.0)

def left_elbow_bend_lock_position():
    print("-----> left_elbow_bend_lock_position")
    pub_left_arm_gripper_finger.publish(3.1)

def left_arm_down():
    print("-----> left_arm_down")
    pub_left_arm_elbow_bend.publish(0.0)
    pub_left_arm_elbow_rotate.publish(0.0)
    pub_left_arm_wrist_rotate.publish(0.0)
    pub_left_arm_lift.publish(0.25) # keep away from body
    pub_left_arm_shoulder.publish(-0.4)
    left_gripper_open_half()

def left_arm_home():
    print("-----> left_arm_home")
    pub_left_arm_lift.publish(0.25)
    pub_left_arm_elbow_rotate.publish(0.0)
    pub_left_arm_elbow_bend.publish(2.0)
    pub_left_arm_wrist_rotate.publish(0.0)
    # pub_left_arm_gripper_finger.publish(0.25)
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
    #pub_left_arm_gripper_finger.publish(0.8)
    pub_left_arm_shoulder.publish(1.0)

def left_arm_extend_full():
    print("-----> left_arm_extend_full")
    pub_left_arm_lift.publish(0.25)
    pub_left_arm_elbow_rotate.publish(0.0)
    pub_left_arm_elbow_bend.publish(0.0)
    pub_left_arm_wrist_rotate.publish(0.0)
    #pub_left_arm_gripper_finger.publish(0.25)
    pub_left_arm_shoulder.publish(3.0)

def left_arm_write_ready():
    print("-----> left_arm_write_ready")
    pub_left_arm_lift.publish(g_left_draw_lift_origin)
    pub_left_arm_elbow_bend.publish(1.57)
    pub_left_arm_wrist_rotate.publish(-1.57)
    pub_left_arm_shoulder.publish(3.0)
    left_pen_up() # does elbow rotate
    left_gripper_close()

def left_arm_draw_left_eye():
    sleep_time = 0.6
    x_line = 0.075
    y_line = 0.050
    print("-----> left_arm_draw_left_eye")
    left_arm_write_ready()
    #time.sleep(2.5)

    left_pen_down()

    pub_left_arm_elbow_bend.publish(1.57 + x_line)
    time.sleep(sleep_time)

    pub_left_arm_lift.publish(g_left_draw_lift_origin - y_line)
    time.sleep(sleep_time)

    pub_left_arm_elbow_bend.publish(1.57 + 0.00)
    time.sleep(sleep_time)

    pub_left_arm_lift.publish(g_left_draw_lift_origin - 0.00)
    time.sleep(sleep_time)

    #pen up
    left_pen_up()

    print("-----> done:")

def left_arm_draw_right_eye():
    sleep_time = 0.6
    x_line = 0.075
    y_line = 0.050
    eye_offset = 0.20
    print("-----> left_arm_draw_right_eye")

    # move to the right eye start position
    pub_left_arm_lift.publish(g_left_draw_lift_origin-(0.00 + eye_offset))
    time.sleep(1.5)

    left_pen_down()
    #time.sleep(1.5)

    # draw
    pub_left_arm_elbow_bend.publish(1.57 + x_line)
    time.sleep(sleep_time)

    pub_left_arm_lift.publish(g_left_draw_lift_origin-(y_line + eye_offset))
    time.sleep(sleep_time)

    pub_left_arm_elbow_bend.publish(1.57 + 0.0)
    time.sleep(sleep_time)

    pub_left_arm_lift.publish(g_left_draw_lift_origin-(0.00 + eye_offset))
    time.sleep(sleep_time)

    #pen up
    left_pen_up()

    print("-----> right eye done:")


def left_arm_draw_mouth():
    sleep_time = 0.5
    mouth_offset = 0.20
    print("-----> left_arm_draw_mouth")

    # move to the mouth start position
    #left_arm_write_ready()
    #time.sleep(2.5)
    pub_left_arm_elbow_bend.publish(1.57 + mouth_offset)
    pub_left_arm_lift.publish(g_left_draw_lift_origin+0.05) # start to the left of the eye
    time.sleep(2.5)

    left_pen_down()

    # draw
    pub_left_arm_elbow_bend.publish(1.57 + mouth_offset + 0.15) #curve down
    time.sleep(0.4)
    pub_left_arm_lift.publish(g_left_draw_lift_origin-0.20) # draw continuous line to the right


    #pub_left_arm_elbow_bend.publish(1.57 + mouth_offset + 0.20) #bottom
    time.sleep(1.0)

    #pub_left_arm_elbow_bend.publish(1.57 + mouth_offset + 0.10) 
    #time.sleep(sleep_time)

    pub_left_arm_elbow_bend.publish(1.57 + mouth_offset) #end of mouth 
    time.sleep(sleep_time)

    #pen up
    left_pen_up()
    print("-----> mouth done")

def draw_smiley():
    print("-----> draw_smiley")

    # go from home position to ready

    #get the elbow tight, to avoid hitting desk
    left_gripper_close()
    pub_left_arm_elbow_bend.publish(2.5)
    time.sleep(0.5) # give time to avoid colliding with desk
    pub_left_arm_lift.publish(g_left_draw_lift_origin)
    pub_left_arm_shoulder.publish(3.0)
    time.sleep(1.0) # give time to avoid colliding with tablet

    pub_left_arm_elbow_bend.publish(1.57)
    pub_left_arm_wrist_rotate.publish(-1.57)
    #time.sleep(1.0) # give time to avoid colliding with tablet
    left_pen_up() # does elbow rotate

    time.sleep(1.5) 
    left_arm_draw_left_eye()
    left_arm_draw_right_eye()
    left_arm_draw_mouth()

    # get ready to move to home position
    pub_left_arm_lift.publish(g_left_draw_lift_origin+0.0)
    pub_left_arm_elbow_rotate.publish(0.0)
    pub_left_arm_elbow_bend.publish(2.8)
    time.sleep(2.8) # give time to avoid colliding with tablet

    print("-----> moving home")
    pub_left_arm_shoulder.publish(1.0)
    time.sleep(1.5) # give time to avoid colliding with tablet
    left_arm_home()

    print("-----> Smiley done")



def left_arm_wave():
    print("-----> left_arm_wave")
    pub_sidetilt.publish(0.0)
    pub_tilt.publish(-0.1)
    pub_pan.publish(0.0)
    pub_left_arm_lift.publish(0.25)
    pub_left_arm_elbow_rotate.publish(0.0)
    pub_left_arm_elbow_bend.publish(2.2)
    pub_left_arm_wrist_rotate.publish(0.0)
    #pub_left_arm_gripper_finger.publish(0.8)
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
# Special Keyboard Commands

def all_home(): # all in home position
    print("-----> all_home")
    head_center()
    left_arm_home()
    right_arm_home()



# ========================================================================================
# Main

if __name__ == '__main__':
    pub_pan = rospy.Publisher('/head_pan_joint/command', Float64, queue_size=1)
    pub_tilt = rospy.Publisher('/head_tilt_joint/command', Float64, queue_size=1)
    pub_sidetilt = rospy.Publisher('/head_sidetilt_joint/command', Float64, queue_size=1)

    pub_left_arm_lift = rospy.Publisher('/left_arm_lift_joint/command', Float64, queue_size=1)
    pub_left_arm_elbow_rotate = rospy.Publisher('/left_arm_elbow_rotate_joint/command', Float64, queue_size=1)
    pub_left_arm_elbow_bend = rospy.Publisher('/left_arm_elbow_bend_joint/command', Float64, queue_size=1)
    pub_left_arm_wrist_rotate = rospy.Publisher('/left_arm_wrist_rotate_joint/command', Float64, queue_size=1)
    pub_left_arm_gripper_finger = rospy.Publisher('/left_arm_gripper_finger_joint/command', Float64, queue_size=1)
    pub_left_arm_shoulder = rospy.Publisher('/left_arm_shoulder_controller/command', Float64, queue_size=1)

    pub_right_arm_lift = rospy.Publisher('/right_arm_lift_joint/command', Float64, queue_size=1)
    pub_right_arm_elbow_rotate = rospy.Publisher('/right_arm_elbow_rotate_joint/command', Float64, queue_size=1)
    pub_right_arm_elbow_bend = rospy.Publisher('/right_arm_elbow_bend_joint/command', Float64, queue_size=1)
    pub_right_arm_wrist_rotate = rospy.Publisher('/right_arm_wrist_rotate_joint/command', Float64, queue_size=1)
    pub_right_arm_gripper_finger = rospy.Publisher('/right_arm_gripper_finger_joint/command', Float64, queue_size=1)
    pub_right_arm_shoulder = rospy.Publisher('/right_arm_shoulder_controller/command', Float64, queue_size=1)



# Initialize node for ROS
rospy.init_node('listener', anonymous=True) # TODO change this!

# Wait for Keyboard command
print();
print("1: head, 11: Right Arm, 21: Left Arm")

while(1):
    key = raw_input("Enter Key (x to eXit): ")
    print("got key: " + key)

    if key == '0':
        all_home()

    elif key == '1':
        head_center()
    elif key == '2':
        right_home()
    elif key == '3':
        left_home()
    elif key == '5':
        head_down()

# RIGHT ARM
    elif key == '10':
        right_arm_home()
    elif key == '11':
        right_arm_down()
    elif key == '12':
        right_get_item()
    elif key == '13':
        right_arm_extend() # give item (gripper stays)
    elif key == '14':
        right_arm_extend_full()
    elif key == '15':
        right_arm_write_ready()
    elif key == '18': 
        right_gripper_open()
    elif key == '19': 
        right_gripper_close()

# LEFT ARM
    elif key == '20':
        left_arm_home()
    elif key == '21':
        left_arm_down()

    elif key == '22':
        left_get_item()
    elif key == '23':
        left_arm_extend() # give item (gripper stays)
    elif key == '24':
        left_arm_extend_full()
    elif key == '25':
        left_arm_write_ready()
    elif key == '28': 
        left_gripper_open()
    elif key == '29': 
        left_gripper_close()

    elif key == '60':
        left_arm_draw_left_eye()
    elif key == '61':
        left_arm_draw_right_eye()
    elif key == '62':
        left_arm_draw_mouth()

# COMPOUND MOVEMENTS
    elif key == '50': # SMILEY!
        draw_smiley()

    elif key == '51': # LEFT WAVE
        left_arm_wave()

    elif key == '90': # put any single publisher test here
        pub_right_arm_elbow_bend.publish(2.9)
        pub_left_arm_elbow_bend.publish(2.9)

    # -----------------------------------------
    elif key == 'q':
        print("-----> key is Quit  -- GOODBYE")
        break
    elif key == 'x':
        print("-----> key is eXit  -- GOODBYE")
        break
    else:
        print("***** UNKNOWN KEY - IGNORED *****")



