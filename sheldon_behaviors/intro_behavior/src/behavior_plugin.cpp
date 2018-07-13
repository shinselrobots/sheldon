// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <behavior_common/behavior.h>
#include <behavior_common/behavior_common.h>
#include <audio_and_speech_common/audio_and_speech_common.h>
#include <std_msgs/Float64.h>
#include <functional>

#define DEFAULT_TILT_ANGLE  (-0.20)

// NOTE: In its current form, this could just be read as text using the Say behavior.  But this
// behavior is planned to be updated to include motion and camera pan/tilt poses to simulate looking
// around (or using the actual person detector to find and talk to someone specifically)

namespace behavior_plugin 
{
    class IntroBehaviorService : public behavior_common::BehaviorActionServiceBase
    {
      public:
        IntroBehaviorService(std::string service_name) :
          behavior_common::BehaviorActionServiceBase(service_name)
        {
          // Publishers for robot servo movements
          head_pan_ = nh_.advertise<std_msgs::Float64>("/head_pan_controller/command", 1); 
          head_tilt_ = nh_.advertise<std_msgs::Float64>("/head_tilt_controller/command", 1);
          head_sidetilt_ = nh_.advertise<std_msgs::Float64>("/head_sidetilt_controller/command", 1);
 
        }

        virtual void StartBehavior(const char *param1, const char *param2)
        {
          if(!speech_.isAvailable())
          {
            ROS_WARN("speech was not available, ending IntroBehaviorService immediately");
            BehaviorComplete(); // Speech not available, so we'll end immediatley
          }
 
          std_msgs::Float64 pos;

          // center head
          pos.data = 0.0;
          head_pan_.publish(pos);
          pos.data = 0.0;
          head_sidetilt_.publish(pos);
          pos.data = DEFAULT_TILT_ANGLE;  
          head_tilt_.publish(pos);
          // boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

          //speech_.speakAndWaitForCompletion("Hello. <break time='500ms'/>");
          speech_.speakAndWaitForCompletion("My name is Sheldon. ");

          pos.data = 0.3;
          head_pan_.publish(pos);
          pos.data = 0.1;
          head_sidetilt_.publish(pos);
          pos.data = DEFAULT_TILT_ANGLE;  
          head_tilt_.publish(pos);
          speech_.speakAndWaitForCompletion("I have been under construction for about a year or so");

          pos.data = -0.3;
          head_pan_.publish(pos);
          pos.data = -0.1;
          head_sidetilt_.publish(pos);
          pos.data = DEFAULT_TILT_ANGLE;  
          head_tilt_.publish(pos);
          speech_.speakAndWaitForCompletion("My brain consists of an Intel core eye 5 processor, connected to five Arduinos for sensory input and control. " );

          pos.data = 0.00;
          head_pan_.publish(pos);
          pos.data = 0.0;
          head_sidetilt_.publish(pos);
          pos.data = DEFAULT_TILT_ANGLE -0.1;  
          head_tilt_.publish(pos);
          speech_.speakAndWaitForCompletion("I have depth cameras and a laser scanner for navigation, and 18 dyna mixel servos for my movements " );

          pos.data = -0.3;
          head_pan_.publish(pos);
          pos.data = -0.1;
          head_sidetilt_.publish(pos);
          pos.data = DEFAULT_TILT_ANGLE;  
          head_tilt_.publish(pos);
          speech_.speakAndWaitForCompletion("my programs are written in sea plus plus and python, on top of Ros, the robot operating system. ");

          pos.data = 0.3;
          head_pan_.publish(pos);
          pos.data = 0.1;
          head_sidetilt_.publish(pos);
          pos.data = DEFAULT_TILT_ANGLE -0.1;  
          head_tilt_.publish(pos);
          speech_.speakAndWaitForCompletion("I love humans, other robots, and dogs, but unfortunately dogs dont seem to like me  <break time='1000ms'/>");

          pos.data = 0.0;
          head_pan_.publish(pos);
          pos.data = 0.0;
          head_sidetilt_.publish(pos);
          pos.data = DEFAULT_TILT_ANGLE;  
          head_tilt_.publish(pos);
          speech_.speakAndWaitForCompletion("the only thing I dont like is darth vaider, he scares me'/>");


 		  BehaviorComplete();
        }

        virtual void PremptBehavior()
        {
          // We were requested to preempt our behavior, so cancel any outstanding 
          // speech immediately.
          speech_.cancel();
        }

        protected:
          audio_and_speech_common::SpeechClient speech_;
          ros::Publisher head_sidetilt_;
          ros::Publisher head_pan_;
          ros::Publisher head_tilt_;

 
    };

    CPP_BEHAVIOR_PLUGIN(IntroPlugin, "/intro_behavior_service", "INTRO", IntroBehaviorService);
  };

PLUGINLIB_EXPORT_CLASS(behavior_plugin::IntroPlugin, behavior_common::BehaviorPlugin);
