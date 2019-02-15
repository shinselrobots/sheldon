#!/usr/bin/env python

head_joints = [
    'head_sidetilt_joint',
    'head_pan_joint',
    'head_tilt_joint',
    ]

right_arm_joints = [
    'right_arm_shoulder_rotate_joint',
    'right_arm_shoulder_lift_joint',
    'right_arm_elbow_rotate_joint',
    'right_arm_elbow_bend_joint',
    'right_arm_wrist_bend_joint',
    'right_arm_wrist_rotate_joint',
    'right_arm_gripper_finger_joint',
    ]

left_arm_joints = [
    'left_arm_shoulder_rotate_joint',
    'left_arm_shoulder_lift_joint',
    'left_arm_elbow_rotate_joint',
    'left_arm_elbow_bend_joint',
    'left_arm_wrist_bend_joint',
    'left_arm_wrist_rotate_joint',
    'left_arm_gripper_finger_joint',
    ]

chest_camera_joints = [
    'chest_camera_tilt_joint',
]

leg_joints = [
    'knee_bend_joint',
    'hip_bend_joint',
    ]

all_servo_joints = head_joints + chest_camera_joints + right_arm_joints + left_arm_joints

all_joints = head_joints + chest_camera_joints + right_arm_joints + left_arm_joints + leg_joints # warning - don't use for dynamixel commands!

right_arm_kinematic_joints =  leg_joints + right_arm_joints
left_arm_kinematic_joints =  leg_joints + left_arm_joints


