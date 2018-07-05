// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <behavior_common/behavior.h>
#include <behavior_common/behavior_common.h>
#include <audio_and_speech_common/audio_and_speech_common.h>
#include <functional>

namespace behavior_plugin 
{
    class TellAJokeBehaviorService : public behavior_common::BehaviorActionServiceBase
    {
      public:
        TellAJokeBehaviorService(std::string service_name) :
          behavior_common::BehaviorActionServiceBase(service_name)
        {
          ros::NodeHandle nh;

          nh.getParam("/jokes", joke_list_);
          if(!joke_list_.empty())
            jokes_.set(joke_list_);

          nh.getParam("/joke_conjunctions", joke_conjunctions_list_);
          if(!joke_conjunctions_list_.empty())
            joke_conjunctions_.set(joke_conjunctions_list_);
        }

        virtual void StartBehavior(const char *param1, const char *param2)
        {
          // If no jokes are loaded, then complete immediately.
          if(joke_list_.empty())
          {
            ROS_WARN("/jokes ROS parameter is empty, ending TellAJokeBehaviorService immediately.");
            BehaviorComplete();
            return;
          }

          // Should we tell just one joke (default), or the number
          // passed to us in param1?  Cap total number of jokes at a time to 4
          num_jokes_to_tell_ = 1;
          if(strlen(param1))
            num_jokes_to_tell_ = atoi(param1);
          if( num_jokes_to_tell_ > 4)
            num_jokes_to_tell_ = 4;
          if(joke_conjunctions_list_.empty())
            num_jokes_to_tell_ = 1;

          // Tell the first joke asynchronously.  When speech is done, we will receive a 
          // speechCompleteCallback below.
          if( false == speech_.speak(jokes_.shuffle_next(), std::bind(&TellAJokeBehaviorService::speechCompleteCallback, this, std::placeholders::_1)))
          {
            ROS_WARN("speech was not available, ending TellAJokeBehaviorService immediately");
            BehaviorComplete(); // Speech not available, so we'll end immediatley
          }
        }

        virtual void PremptBehavior()
        {
          // We were requested to preempt our behavior, so cancel any outstanding 
          // speech immediately.
          speech_.cancel();
        }

        void speechCompleteCallback(bool success)
        {
          if(success)
          {
            if(--num_jokes_to_tell_)
            {
              // Tell another joke, after a small pause
              ros::Duration(2.0).sleep();
              if( speech_.speakAndWaitForCompletion(joke_conjunctions_.shuffle_next()))
              {
                // Previous sync speech command was not cancelled by our PreemptBehavior routine,
                // so we'll continue with the next joke asynchronously.
                ros::Duration(1.0).sleep();
                speech_.speak(jokes_.shuffle_next(), std::bind(&TellAJokeBehaviorService::speechCompleteCallback, this, std::placeholders::_1));
              }
            }
            else
              // We have no more jokes to tell, so complete our behavior
              BehaviorComplete();
          }
        }

        protected:
          audio_and_speech_common::SpeechClient speech_;
          audio_and_speech_common::speech_indexer jokes_;
          audio_and_speech_common::speech_indexer joke_conjunctions_;
          std::vector<std::string> joke_list_;
          std::vector<std::string> joke_conjunctions_list_;
          int num_jokes_to_tell_;
    };

    CPP_BEHAVIOR_PLUGIN(TellAJokeBehaviorPlugin, "/tell_a_joke_behavior_service", "TELL_A_JOKE", TellAJokeBehaviorService);
  };

PLUGINLIB_EXPORT_CLASS(behavior_plugin::TellAJokeBehaviorPlugin, behavior_common::BehaviorPlugin);
