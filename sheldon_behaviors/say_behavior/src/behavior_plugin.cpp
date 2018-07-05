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
    class SayBehaviorService : public behavior_common::BehaviorActionServiceBase
    {
      public:
        SayBehaviorService(std::string service_name) :
          behavior_common::BehaviorActionServiceBase(service_name)
        {
        }

        virtual void StartBehavior(const char *param1, const char *param2)
        {
          // Speak the content provided in param1, then complete the behavior
          if( false == speech_.speak(param1, std::bind(&SayBehaviorService::speechCompleteCallback, this, std::placeholders::_1)))
          {
            ROS_WARN("speech was not available, ending SayBehavior immediately");
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
          // Speech is complete
          if(success)
            BehaviorComplete();
        }

        protected:
          audio_and_speech_common::SpeechClient speech_;
    };

    CPP_BEHAVIOR_PLUGIN(SayPlugin, "/say_behavior_service", "SAY", SayBehaviorService);
  };

PLUGINLIB_EXPORT_CLASS(behavior_plugin::SayPlugin, behavior_common::BehaviorPlugin);
