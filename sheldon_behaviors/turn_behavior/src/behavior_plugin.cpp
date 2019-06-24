// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <behavior_common/behavior.h>
#include <behavior_common/behavior_common.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <angles/angles.h>
#include <string>

// fastest speed robot is capable of moving: 
  //const double MAX_SPEED_METERS_PER_SECOND = 3.0; 
  //const double MIN_SPEED_METERS_PER_SECOND = 0.5; // OK?
// fastest turn robot is capable of:
  //const double MAX_TURN_RADIANS_PER_SECOND = 8.0; // I THINK THIS IS WRONG NOW 
  const double MIN_TURN_RADIANS_PER_SECOND = 0.04;  // 0.4
  const double STALL_RADIANS_PER_SECOND = 0.025;  // 0.25 Don't stall during turn
// (Note: From Wheel_Control to Sabertooth, speeds are 0.0 - 1.0)

namespace behavior_plugin 
{

  enum State
  {
  	idle,
  	initializing_start_position,
  	turning,
    ending
  };
	
  class TurnService : public behavior_common::BehaviorActionServiceBase
	{
  public:
    TurnService(std::string service_name) :
      behavior_common::BehaviorActionServiceBase(service_name),
      state_(idle),
      DEFAULT_SPEED(0.15),
      rotation_speed_(DEFAULT_SPEED),
      ramp_down_fudge_(0.0),
      turn_progress_(0.0),
      goal_turn_amount_(0.0),    
      last_angle_(0.0)   
    {
      // Init publishers and subscribers
      // rotation is monitored in odometryCallback
      cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("move_base/priority1", 1);  // Low Priority
    }

    virtual ~TurnService()
    {
      // Stop moving before exiting
      state_ = idle;    
      stop();
    }

    virtual void StartBehavior(const char *param1, const char *param2)
    {
      // Implementation of behavior here.  You can either
      // register additional callbacks which will be invoked while your behavior
      // is running, or you can run the entire behavior here (watching for a
      // possible request to prempt your behavior below)
 
      // Param1: amount in degrees ()
      // Param2: speed
      ROS_INFO("TURN BEHAVIOR: Amt = %s (degrees), Speed = %s (Rad/Sec) range: 1.0-8.0", param1, param2);

      double goal_turn_amount_degrees = atof(param1);
      goal_turn_amount_ = angles::from_degrees(goal_turn_amount_degrees);

      rotation_speed_ = fabs(atof(param2));
      // Kludge to compensate for velocity smoother ramp down
      ramp_down_fudge_ = angles::from_degrees(10 * rotation_speed_);  // speed is 1.0 -> 8.0

      if(ramp_down_fudge_ > (fabs(goal_turn_amount_) / 2.0)) 
      {
         ramp_down_fudge_ = (fabs(goal_turn_amount_) / 2.0);  // limit ramp down to half of full turn amount
      }


      // ramp_down_fudge_ = angles::from_degrees(10.0 * (10 * rotation_speed_));  // speed is 0.0 -> 1.0

      if( rotation_speed_ < MIN_TURN_RADIANS_PER_SECOND )
      {
        ROS_WARN("TURN BEHAVIOR: Warning!  Speed Too Slow!");
      }

      ROS_INFO("TURN BEHAVIOR: Goal turn amount (rad, deg): %f, %f", goal_turn_amount_, goal_turn_amount_degrees);
      ROS_INFO("TURN BEHAVIOR: Turn speed: %f", rotation_speed_);
      odometrySubscriber_ = nh_.subscribe("/odom", 1, &TurnService::odometryCallback, this);
      state_ = initializing_start_position;
    }

    virtual void PremptBehavior()
    {
      // TODO: Add code which prempts your running behavior here.
      state_ = idle;
      stop();
      odometrySubscriber_.shutdown();
    }

    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
      // Called on each odometry update (~50hz)
      double yaw, pitch, roll;

      if(state_ != idle)
      {
        ROS_INFO("TURN BEHAVIOR:  --------------------------------------------");
        ROS_INFO("TURN BEHAVIOR:  odometryCallback called");
			}

      #if 0
      ROS_INFO("Pose Orientation: x: [%f], y: [%f], z: [%f], w: [%f]", 
	      msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, 
	      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
      #endif

      tf::Quaternion q(
	      msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, 
	      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
      tf::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);

      if(initializing_start_position == state_)
      {
        ROS_INFO("TURN BEHAVIOR: RAW Start Angle: %f radians, %f degrees", yaw, angles::to_degrees(yaw));
        turn_progress_ = 0.0;
        last_angle_ = angles::normalize_angle_positive(yaw); //  0-360
        ROS_INFO("TURN BEHAVIOR: Start Angle: %f radians, %f degrees", last_angle_, angles::to_degrees(last_angle_));

        // Start moving
        publish_turn(rotation_speed_);

        state_ = turning;
        return;

      }
      else if(turning == state_)
      {
        // Monitor movement, stop when within goal postion
        double current_angle = angles::normalize_angle_positive(yaw); //  0-360
        double angle_delta = angles::shortest_angular_distance(last_angle_, current_angle);
        turn_progress_ += angle_delta;
        last_angle_ = current_angle;
        double turn_remaining = fabs(goal_turn_amount_) - (fabs(turn_progress_) );

        //if( fabs(turn_progress_) > 0.03) // display once turn actually starts
        {
  	  	  ROS_INFO("TURN BEHAVIOR last_angle_ = %f, current_angle = %f, delta = %f deg", 
            angles::to_degrees(last_angle_), angles::to_degrees(current_angle), angles::to_degrees(angle_delta));

  	  	  ROS_INFO("TURN BEHAVIOR Progress: %f rad, %f deg", 
            turn_progress_, angles::to_degrees(turn_progress_));

          ROS_INFO("TURN Remaining: %f rad, %f deg, Rampdown at %f rad, %f deg", 
            turn_remaining, angles::to_degrees(turn_remaining), ramp_down_fudge_, angles::to_degrees(ramp_down_fudge_) );

          ROS_INFO("    DBG: Heading: %f radians, %f degrees", yaw, angles::to_degrees(yaw));
        }

        // See if it's time to stop
        if(turn_remaining < ramp_down_fudge_)
        {
	      ROS_INFO("TURN BEHAVIOR: Turn almost complete!  Stopping.");
          stop();
          state_ = ending;
        }
        else if(turn_remaining < (2.0 * ramp_down_fudge_))
        {
  	      ROS_INFO("TURN BEHAVIOR: Slowing...");
          publish_turn(rotation_speed_ / 3.0);
        }
        else if( turn_remaining < fabs(goal_turn_amount_ / 2.0) )
        {
	        ROS_INFO("TURN BEHAVIOR: Slowing at half way...");
          publish_turn(rotation_speed_ / 2.0);
        }
        else
        {
          // continue to turn
          publish_turn(rotation_speed_);
        }

      } // turning == state_
      else if (ending == state_)
      {
        ROS_INFO("TURN BEHAVIOR: Final Heading: %f radians, %f degrees", yaw, angles::to_degrees(yaw));

        double current_angle = angles::normalize_angle_positive(yaw); //  0-360
        double angle_delta = angles::shortest_angular_distance(last_angle_, current_angle);
        turn_progress_ += angle_delta;
	  	  ROS_INFO("TURN BEHAVIOR: Final Turn Amount: %f rad, %f deg", 
          turn_progress_, angles::to_degrees(turn_progress_) );

    	  // Mark behavior complete
        BehaviorComplete();  
        state_ = idle;
      }
    } // odometryCallback

    // Utility Functions

    void stop()
    {
      geometry_msgs::Twist cmd;
      cmd.linear.x = cmd.angular.z = 0;  
      cmd_vel_pub_.publish(cmd); 
    }

    void publish_turn(double turn_speed)
    {
      if(turn_speed < STALL_RADIANS_PER_SECOND) 
      {
        turn_speed = STALL_RADIANS_PER_SECOND;
      }

      if(goal_turn_amount_ < 0.0)
      {
        turn_speed *= -1.0;
      }

      geometry_msgs::Twist cmd;
      cmd.linear.x = 0;
      cmd.angular.z = turn_speed;  
      cmd_vel_pub_.publish(cmd);
    }

   
/***
    double NormalizeAngleOld(double angle)
    {
      double normalized = angle;
      while(normalized > pi) 
      {
          normalized -= pi * 2.0;
      }
      while(normalized < -pi)
      {
        normalized += pi * 2.0; 
      }
      return normalized;
    }
***/


  protected:
    ros::Subscriber odometrySubscriber_;
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;

    State  state_;
    const double DEFAULT_SPEED;
    double rotation_speed_;
    double ramp_down_fudge_;
    double turn_progress_;
    double goal_turn_amount_;
    double last_angle_;
 

  }; // TurnService Class

  CPP_BEHAVIOR_PLUGIN(TurnBehaviorPlugin, "/turn_service", "TURN", TurnService);
}; // namespace behavior_plugin

PLUGINLIB_EXPORT_CLASS(behavior_plugin::TurnBehaviorPlugin, behavior_common::BehaviorPlugin);
