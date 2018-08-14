// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <behavior_common/behavior.h>
#include <behavior_common/behavior_common.h>
#include <audio_and_speech_common/audio_and_speech_common.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <functional>
#include <time.h>

static std::vector<std::string> ack_text = {
  "Lets see", 
  "well, lets see", 
  "let me check", 
  "let me see" 
};

static std::vector<std::string> final_text = {
  "Was that really a good use of 2.6 billion transistors?", 
  "I have 5 thousand lines of code, so you can ask me the time",
  //"My watch is synchronized with the atomic clock in Boulder Colorado",
  "I think perhaps I am the world's most expensive clock"
};

namespace behavior_plugin 
{
    class TimeBehaviorService : public behavior_common::BehaviorActionServiceBase
    {
      public:
        TimeBehaviorService(std::string service_name) :
          behavior_common::BehaviorActionServiceBase(service_name)
        {
          s1_.set(ack_text);
          s2_.set(final_text);

          // Publish robot servo movements
          head_pan_ = nh_.advertise<std_msgs::Float64>("/head_pan_joint/command", 1); 
          head_tilt_ = nh_.advertise<std_msgs::Float64>("/head_tilt_joint/command", 1);
          head_sidetilt_ = nh_.advertise<std_msgs::Float64>("/head_sidetilt_joint/command", 1);
          right_arm_shoulder_rotate_ = nh_.advertise<std_msgs::Float64>("/right_arm_shoulder_rotate_joint/command", 1);
          right_arm_elbow_rotate_ = nh_.advertise<std_msgs::Float64>("/right_arm_elbow_rotate_joint/command", 1);
          right_arm_elbow_bend_ = nh_.advertise<std_msgs::Float64>("/right_arm_elbow_bend_joint/command", 1);
 
          // enable/disable microphone when robot is moving servos.  
          // (Note system_enable vs. speech_enable vs. user_enable)
          mic_system_enable_ = nh_.advertise<std_msgs::Bool>("/microphone/system_enable", 1);


        }

        virtual void StartBehavior(const char *param1, const char *param2)
        {
          std_msgs::Float64 pos;
          std_msgs::Bool mic_enable_msg;

          if(!speech_.isAvailable())
          {
            ROS_WARN("speech was not available, ending TimeBehaviorService immediately");
            BehaviorComplete(); // Speech not available, so we'll end immediatley
          }

          time_t rawtime;
          struct tm * timeinfo;
          std::ostringstream stream;

          speech_.speakAndWaitForCompletion(s1_.shuffle_next());
       
          // Look forward 
          pos.data = 0.0;
          head_pan_.publish(pos);
          head_tilt_.publish(pos);
          head_sidetilt_.publish(pos);
          
          // Mute the mic
          mic_enable_msg.data = false;
          mic_system_enable_.publish(mic_enable_msg);

         // move arm forward
		  pos.data = 0.7; 
          right_arm_shoulder_rotate_.publish(pos);
		  pos.data = 1.5708; // 90 degrees
          right_arm_elbow_bend_.publish(pos);
          boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

          pos.data = 0.0;
          head_pan_.publish(pos);
          pos.data = 0.75; // look down at wrist
          head_tilt_.publish(pos);
          boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

          // rotate elbow and look at wrist
		  pos.data = -1.4;
          right_arm_elbow_rotate_.publish(pos);
          boost::this_thread::sleep(boost::posix_time::milliseconds(3000));

          // start moving head up and hand back
		  pos.data = 0.0;
          right_arm_elbow_rotate_.publish(pos);
          pos.data = 0.0;
          head_pan_.publish(pos);
          pos.data = 0.0;
          head_tilt_.publish(pos);
          boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

          // Determine the actual time and say it
          time ( &rawtime );
          timeinfo = localtime ( &rawtime );

          stream << "The time is ";
          if( timeinfo->tm_hour > 12 )
            // todo - fix "3:03" said as "3 3 PM"
            // stream << (timeinfo->tm_hour - 12) << " " << setfill('o') << setw(2) << timeinfo->tm_min << " PM";
            stream << (timeinfo->tm_hour - 12) << " " 
              << std::fixed << std::setw( 2 ) << timeinfo->tm_min << " PM";
          else
            stream << timeinfo->tm_hour << " " 
              << std::fixed << std::setw( 2 ) << timeinfo->tm_min << " AM";
          speech_.speakAndWaitForCompletion(stream.str());
 
            ROS_INFO_STREAM("REPORTING TIME AS: [" << stream.str() << "]" );


          // Move back to home position

          pos.data = 0.0;
          head_tilt_.publish(pos);
		  pos.data = 0.0;
          head_pan_.publish(pos);
		  pos.data = -0.5; // Home
          right_arm_shoulder_rotate_.publish(pos);
		  pos.data = 2.2; // Home
          right_arm_elbow_bend_.publish(pos);

          // Finalize behavior
          speech_.speakAndWaitForCompletion(s2_.shuffle_next());
          boost::this_thread::sleep(boost::posix_time::milliseconds(3000));

          // Un-Mute the mic
          mic_enable_msg.data = true;
          mic_system_enable_.publish(mic_enable_msg);

		  BehaviorComplete();


       }

        virtual void PremptBehavior()
        {
          // We were requested to preempt our behavior, so cancel any outstanding 
          // speech immediately.
          speech_.cancel();
        }

        protected:
          ros::NodeHandle nh_;
          audio_and_speech_common::SpeechClient speech_;
          audio_and_speech_common::speech_indexer s1_, s2_;
          //ros::Publisher pan_;
          //ros::Publisher tilt_;


          ros::Publisher head_sidetilt_;
          ros::Publisher head_pan_;
          ros::Publisher head_tilt_;

          //ros::Publisher right_arm_lift_;
          ros::Publisher right_arm_elbow_rotate_;
          ros::Publisher right_arm_elbow_bend_;
          ros::Publisher right_arm_wrist_rotate_;
          ros::Publisher right_arm_gripper_;
          ros::Publisher right_arm_shoulder_rotate_;
          ros::Publisher mic_system_enable_;

    };

    CPP_BEHAVIOR_PLUGIN(TimePlugin, "/time_behavior_service", "TELL_TIME", TimeBehaviorService);
  };

PLUGINLIB_EXPORT_CLASS(behavior_plugin::TimePlugin, behavior_common::BehaviorPlugin);
