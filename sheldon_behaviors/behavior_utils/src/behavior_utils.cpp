// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

//#include <Python.h>
//#include <unistd.h>
#include <stdio.h>
#include <sys/types.h>
#include <signal.h>
#include <wait.h>
#include <behavior_utils/behavior_utils.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>


namespace sheldon_behavior_utils
{

  static float RandomFloat(float min, float max)
  {
      float r = (float)rand() / (float)RAND_MAX;
      return min + r * (max - min);
  }


  BehaviorUtils::BehaviorUtils() //:
    /// TODO head(0)
    //as_(nh_, name, false),
    //action_name_(name)
  {
    ROS_INFO("BehaviorUtils Initializing");


    // NEW STUFF
    head_pan_speed_sc_ = 
      nh_.serviceClient<dynamixel_controllers::SetSpeed>("/head_pan_controller/set_speed");
    head_tilt_speed_sc_ = 
      nh_.serviceClient<dynamixel_controllers::SetSpeed>("/head_tilt_controller/set_speed");
    head_sidetilt_speed_sc_ = 
      nh_.serviceClient<dynamixel_controllers::SetSpeed>("/head_sidetilt_controller/set_speed");


    //register client actions
    head = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("head_controller/follow_joint_trajectory");
    leftArm = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("left_arm_controller/follow_joint_trajectory");
    rightArm = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("right_arm_controller/follow_joint_trajectory");

    // Wait for client server registrations above to complete in parallel
    ROS_INFO("BehaviorUtils: waiting for services");
    int retries = 0;
    do
    {
        if(head->isServerConnected() && leftArm->isServerConnected() && rightArm->isServerConnected() )
            break;

        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    }
    while(retries++<5);

    
    if(!head->waitForServer(ros::Duration(0.1)))
    {
      ROS_WARN("BehaviorUtils: head controller not present, skipping");
      delete head; head = 0;
    }

    if(!leftArm->waitForServer(ros::Duration(0.1)))
    {
      ROS_WARN("BehaviorUtils: leftArm controller not present, skipping");
      delete leftArm; leftArm = 0;
    }
 
    if(!rightArm->waitForServer(ros::Duration(0.1)))
    {
      ROS_WARN("BehaviorUtils: rightArm controller not present, skipping");
      delete rightArm; rightArm = 0;
    }

    waist_calibrate_ = nh_.advertise<std_msgs::Empty>("/waist_calibrate", 1);
    waist_pos_ = nh_.advertise<std_msgs::Float32>("/waist_goal_position", 1);

  ROS_INFO("BehaviorUtils Initialization complete");

  }

	BehaviorUtils::~BehaviorUtils()
	{
		if(head) delete head;
	  if(leftArm) delete leftArm;
	  if(rightArm) delete rightArm;
	}



  //////////////////////////////////////////////////////////////////////////////////////
  // HEAD

  void BehaviorUtils::SetHeadPanSpeed(float speed)
  {
    dynamixel_controllers::SetSpeed srv;
    srv.request.speed = speed;
    if(head_pan_speed_sc_.call(srv))
    {
      ROS_INFO("SetHeadPanSpeed SUCCESS");
    }
    else
    {
      ROS_WARN("SetHeadPanSpeed FAILED");
    }
  }

  void BehaviorUtils::SetHeadTiltSpeed(float speed)
  {
    dynamixel_controllers::SetSpeed srv;
    srv.request.speed = speed;
    if(head_tilt_speed_sc_.call(srv))
    {
      ROS_INFO("SetHeadTiltSpeed SUCCESS");
    }
    else
    {
      ROS_WARN("SetHeadTiltSpeed FAILED");
    }
  }

  void BehaviorUtils::SetHeadSideTiltSpeed(float speed)
  {
    dynamixel_controllers::SetSpeed srv;
    srv.request.speed = speed;
    if(head_sidetilt_speed_sc_.call(srv))
    {
      ROS_INFO("SetHeadSideTiltSpeed SUCCESS");
    }
    else
    {
      ROS_WARN("SetHeadSideTiltSpeed FAILED");
    }
  }



  //////////////////////////////////////////////////////////////////////////////////////
  // IS THIS OLD STUFF?  USES FollowJointTrajectoryGoal, which I think is slow/clunky?
  //////////////////////////////////////////////////////////////////////////////////////

  // HEAD CONTROL

  void BehaviorUtils::headHome()
  {
    ROS_INFO( "BehaviorUtils::headHome called" );

    // Sidetilt, Tilt, Pan, MoveTime(Seconds), waitForCompletion 
    moveHead(0.0, HEAD_TILT_HOME, 0.0, 2.0, false);

  }

  void BehaviorUtils::moveHead(float sidetilt, float tilt, float pan, float intervalSeconds, bool waitForCompletion)
  {  
    ROS_INFO("BehaviorUtils::moveHead called");

    control_msgs::FollowJointTrajectoryGoal goal;

    goal.trajectory.joint_names.resize(3);
    goal.trajectory.joint_names[0] = "head_sidetilt_joint";
    goal.trajectory.joint_names[1] = "head_tilt_joint";
    goal.trajectory.joint_names[2] = "head_pan_joint";
    

    goal.trajectory.points.resize(1);
    goal.trajectory.points[0].positions.resize(3);
    goal.trajectory.points[0].positions[0] = sidetilt;
    goal.trajectory.points[0].positions[1] = tilt;
    goal.trajectory.points[0].positions[2] = pan;
    goal.trajectory.points[0].time_from_start = ros::Duration(intervalSeconds);

    head->sendGoal(goal);
    if(waitForCompletion)
      head->waitForResult();
  }


  // RIGHT ARM CONTROL

  void BehaviorUtils::rightArmHome()
  {
    ROS_INFO( "BehaviorUtils::rightArmHome called" );
    // shoulder_rotate, shoulder_lift, elbow_rotate, elbow_bend, wrist_rotate, gripper_open, intervalSeconds, waitForCompletion
    moveRightArm(-1.0, 0.25, 0.0, 2.2, 0.0, 0.25, 3.0 /*Seconds*/, false);

  }

  void BehaviorUtils::moveRightArm(float shoulder_rotate, float shoulder_lift, float elbow_rotate, float elbow_bend, float wrist_rotate, float gripper_open, float intervalSeconds, bool waitForCompletion)
  {
    //ROS_INFO("BehaviorUtils::moveRightArm");

    control_msgs::FollowJointTrajectoryGoal goal;

    goal.trajectory.joint_names.resize(6);
    goal.trajectory.joint_names[0] = "right_arm_shoulder_rotate_joint";
    goal.trajectory.joint_names[1] = "right_arm_shoulder_lift_joint";
    goal.trajectory.joint_names[2] = "right_arm_elbow_rotate_joint";
    goal.trajectory.joint_names[3] = "right_arm_elbow_bend_joint";
    goal.trajectory.joint_names[4] = "right_arm_wrist_rotate_joint";
    goal.trajectory.joint_names[5] = "right_arm_gripper_joint";

    goal.trajectory.points.resize(1);
    goal.trajectory.points[0].positions.resize(6);
    goal.trajectory.points[0].positions[0] = shoulder_rotate;
    goal.trajectory.points[0].positions[1] = shoulder_lift; // shoulder lift SERVO BROKEN
    goal.trajectory.points[0].positions[2] = elbow_rotate;
    goal.trajectory.points[0].positions[3] = elbow_bend;
    goal.trajectory.points[0].positions[4] = wrist_rotate;
    goal.trajectory.points[0].positions[5] = gripper_open;
    goal.trajectory.points[0].time_from_start = ros::Duration(intervalSeconds);

    rightArm->sendGoal(goal);
    if(waitForCompletion)
      rightArm->waitForResult();
  }


  // LEFT ARM CONTROL

  void BehaviorUtils::leftArmHome()
  {
    ROS_INFO( "BehaviorUtils::rightArmHome called" );
    // shoulder_rotate, shoulder_lift, elbow_rotate, elbow_bend, wrist_rotate, gripper_open, intervalSeconds, waitForCompletion
    moveLeftArm(-1.0, 0.25, 0.0, 2.2, 0.0, 0.25, 3.0 /*Seconds*/, false);

  }

  void BehaviorUtils::moveLeftArm(float shoulder_rotate, float shoulder_lift, float elbow_rotate, float elbow_bend, float wrist_rotate, float gripper_open, float intervalSeconds, bool waitForCompletion)
  {  
    //ROS_INFO("BehaviorUtils::moveLeftArm");

    control_msgs::FollowJointTrajectoryGoal goal;

    goal.trajectory.joint_names.resize(6);
    goal.trajectory.joint_names[0] = "left_arm_shoulder_rotate_joint";
    goal.trajectory.joint_names[1] = "left_arm_shoulder_lift_joint";
    goal.trajectory.joint_names[2] = "left_arm_elbow_rotate_joint";
    goal.trajectory.joint_names[3] = "left_arm_elbow_bend_joint";
    goal.trajectory.joint_names[4] = "left_arm_wrist_rotate_joint";
    goal.trajectory.joint_names[5] = "left_arm_gripper_joint";

    goal.trajectory.points.resize(1);
    goal.trajectory.points[0].positions.resize(6);
    goal.trajectory.points[0].positions[0] = shoulder_rotate;
    goal.trajectory.points[0].positions[1] = shoulder_lift;
    goal.trajectory.points[0].positions[2] = elbow_rotate;
    goal.trajectory.points[0].positions[3] = elbow_bend;
    goal.trajectory.points[0].positions[4] = wrist_rotate;
    goal.trajectory.points[0].positions[5] = gripper_open;
    goal.trajectory.points[0].time_from_start = ros::Duration(intervalSeconds);

    leftArm->sendGoal(goal);
    if(waitForCompletion)
        leftArm->waitForResult();
  }

  // WAIST CONTROL
  void BehaviorUtils::bowWaistDown(bool waitForCompletion)
{
    std_msgs::Float32 pos;
    pos.data = 0.8;
    waist_pos_.publish(pos); 

    boost::this_thread::sleep(boost::posix_time::milliseconds(2500));
}

void BehaviorUtils::bowWaistUp(bool waitForCompletion)
{
    std_msgs::Float32 pos;
    pos.data = 0.0;
    waist_pos_.publish(pos); 

    boost::this_thread::sleep(boost::posix_time::milliseconds(2500));
}


void BehaviorUtils::bowWaist(bool waitForCompletion)
{
    std_msgs::Float32 pos;
    pos.data = 0.8;
    waist_pos_.publish(pos); 

    boost::this_thread::sleep(boost::posix_time::milliseconds(2500));

    pos.data = 0.0;
    waist_pos_.publish(pos); 

    boost::this_thread::sleep(boost::posix_time::milliseconds(2500));
}



}
