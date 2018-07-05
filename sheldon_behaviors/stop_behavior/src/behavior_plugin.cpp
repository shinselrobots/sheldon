// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

// This behavior will do a "Raw" move, meaning, it does not use sensors to determine path or safety!  It just moves!

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <behavior_common/behavior.h>
#include <behavior_common/behavior_common.h>
#include <audio_and_speech_common/audio_and_speech_common.h>
#include <nav_msgs/Odometry.h>
#include <string>


namespace behavior_plugin 
{

  class StopService : public behavior_common::BehaviorActionServiceBase
  {
  public:
    StopService(std::string service_name) :
      behavior_common::BehaviorActionServiceBase(service_name)
    {
      // Use safety_control for high priority.  Stop rules all!
      cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/safety_control", 1); 
    }

    virtual ~StopService()
    {
    }

    virtual void StartBehavior(const char *param1, const char *param2)
    {
      ROS_INFO("STOP called");
      stop();

      if(!speech_.isAvailable())
      {
        ROS_WARN("STOP behavior: speech was not available");
      }
      else
      {
        speech_.speakAndWaitForCompletion("Stopping");
      }
      BehaviorComplete();
    }

    virtual void PremptBehavior()
    {
      // Request to preempt behavior, so cancel any outstanding speech.
      // speech_.cancel();
    }

    // Utility Functions

    void stop()
    {
      geometry_msgs::Twist cmd;
      cmd.linear.x = cmd.angular.z = 0;  
      cmd_vel_pub_.publish(cmd); 
    }


  protected:
    ros::NodeHandle nh_;
    audio_and_speech_common::SpeechClient speech_;
    // audio_and_speech_common::speech_indexer s1_, s2_;
    ros::Publisher cmd_vel_pub_;

  };

    CPP_BEHAVIOR_PLUGIN(StopBehaviorPlugin, "/stop_service", "STOP", StopService);
};

PLUGINLIB_EXPORT_CLASS(behavior_plugin::StopBehaviorPlugin, behavior_common::BehaviorPlugin);
