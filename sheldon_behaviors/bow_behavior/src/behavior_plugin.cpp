// Bow Behavior

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <behavior_common/behavior.h>
#include <behavior_common/behavior_common.h>
#include <behavior_utils/behavior_utils.h>
#include <audio_and_speech_common/audio_and_speech_common.h>
#include <std_msgs/Float64.h>
#include <functional>
#include <time.h>

namespace behavior_plugin 
{
  class BowBehaviorService : public behavior_common::BehaviorActionServiceBase
  {
    public:
      BowBehaviorService(std::string service_name) :
        behavior_common::BehaviorActionServiceBase(service_name)
      {
          // Publishers for robot servo movements
          head_pan_ = nh_.advertise<std_msgs::Float64>("/head_pan_controller/command", 1); 
          head_tilt_ = nh_.advertise<std_msgs::Float64>("/head_tilt_controller/command", 1);
          head_sidetilt_ = nh_.advertise<std_msgs::Float64>("/head_sidetilt_controller/command", 1);
          //right_arm_shoulder_rotate_ = nh_.advertise<std_msgs::Float64>("/right_arm_shoulder_lift_controller/command", 1);
          //right_arm_elbow_rotate_ = nh_.advertise<std_msgs::Float64>("/right_arm_elbow_rotate_controller/command", 1);
 
      }

      virtual void StartBehavior(const char *param1, const char *param2)
      {
        std_msgs::Float64 pos;

        if(!speech_.isAvailable())
        {
          ROS_WARN("speech is not available");
        }

        // center head
        pos.data = 0.0;
        head_pan_.publish(pos);
        head_sidetilt_.publish(pos);

        pos.data = 0.0;  
        head_tilt_.publish(pos);
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

        //speech_.speakAndWaitForCompletion("It is nice to meet you");
        speech_.speakAndWaitForCompletion("oh");
        //speech_.speakAndWaitForCompletion("conebaan wah");
        //speech_.speakAndWaitForCompletion("washang how");


          // move arms into position (TODO)
        //pos.data = 0.7;
        //right_arm_shoulder_rotate_.publish(pos);
        //pos.data = 1.5708; // 90 degrees
        //right_arm_elbow_bend_.publish(pos);
        //boost::this_thread::sleep(boost::posix_time::milliseconds(2000));



         utils_.bowWaist(true);
        //utils_.readyPositions();

        //utils_.headHome();
    
        // shoulder_rotate, shoulder_lift, elbow_rotate, elbow_bend, wrist_rotate, claw
        //utils_.moveRightArm(-1.0, 0.25, 0.0, 2.2, 0.0, 0.25, 3.0 /*Seconds*/, false);

        // shoulder_rotate, shoulder_lift, elbow_rotate, elbow_bend, wrist_rotate, claw
        //utils_.moveLeftArm(-1.0, 0.25, 0.0, 2.2, 0.0, 0.25, 3.0 /*Seconds*/, false);

        //speech_.speakAndWaitForCompletion("clear objects close to me");

        // allow time for servos to move into position
        //boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
        speech_.speakAndWaitForCompletion("thank you, you are too kind");
        
		BehaviorComplete();
        // Done!
      }

      virtual void PremptBehavior()
      {
        // We were requested to preempt our behavior, so cancel any outstanding 
        // speech immediately.
        speech_.cancel();
      }

      protected:
        ros::NodeHandle nh_;
        sheldon_behavior_utils::BehaviorUtils utils_;
        audio_and_speech_common::SpeechClient speech_;
        //audio_and_speech_common::speech_indexer s1_, s2_;

        ros::Publisher head_sidetilt_;
        ros::Publisher head_pan_;
        ros::Publisher head_tilt_;
       

  };

  CPP_BEHAVIOR_PLUGIN(BowPlugin, "/bow_behavior_service", "BOW", BowBehaviorService);
};

PLUGINLIB_EXPORT_CLASS(behavior_plugin::BowPlugin, behavior_common::BehaviorPlugin);

