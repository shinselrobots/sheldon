#ifndef __SHELDON_BEHAVIOR_UTILS_H__
#define __SHELDON_BEHAVIOR_UTILS_H__


#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp> 
#include <sys/types.h>
#include <dynamixel_controllers/SetSpeed.h>


namespace sheldon_behavior_utils 
{
  #define HEAD_TILT_HOME (-0.25) // Look Up Slightly
  #define SHOULDER_ROTATE_SLEEP (-0.25) // straight down, no load


  // TODO - MOVE THIS TO ANOTHER HEADER FILE (or better yet, YAML)
  // MOVE ARMS - All of below are in the form:
  //                                    right_arm_shoulder_lift_joint', 'right_arm_elbow_rotate_joint', 'left_arm_claw_joint', 'right_arm_elbow_bend_joint', 'left_arm_elbow_rotate_joint', 'right_arm_wrist_joint', 'right_arm_claw_joint', 'head_roll_joint', 'left_arm_shoulder_lift_joint', 'left_arm_elbow_bend_joint', 'head_tilt_joint', 'right_arm_shoulder_rotate_joint', 'left_arm_wrist_joint', 'left_arm_shoulder_rotate_joint'
  #define MOVE_POSITION_HELLO           0.05522330836388308, 0.1303883669702795, 0.006135923151542565, 2.1690488340702965, -0.2500388684253595, -0.21322332951610412, 0.11351457830353745, HEAD_GLOBAL_Y, 0.039883500485026674, 1.9773012355845914, 0.0000000000000000, -0.019941750242513337, -0.0046019423636569235, 0.056757289151768725
  #define MOVE_POSITION_SELF_REFERENCE  0.09203884727313846, -0.8958447801252144, 0.006135923151542565, 2.366932355707544, 0.15033011721279282, -0.3328738309711841, -0.0337475773334841, HEAD_GLOBAL_Y, 0.0337475773334841, 2.072408044433501, 0.0000000000000000, 0.23930100291016002, 0.0030679615757712823, -0.34514567727426926
  #define MOVE_POSITION_BOW1            0.34974761963792617, -0.29605829206192874, 0.0966407896367954, 2.184388641949153, 0.28225246497095796, -0.1518640980006785, -0.021475731030398976, HEAD_GLOBAL_Y, 0.3175340230923277, 2.086213871524472, 0.0000000000000000, -0.7731263170943632, 0.056757289151768725, -0.8068738944278473
  #define MOVE_POSITION_BOW2            0.34974761963792617, -0.29605829206192874, 0.0966407896367954, 2.184388641949153, 0.28225246497095796, -0.1518640980006785, -0.021475731030398976, 0.5000000000000000, 0.3175340230923277, 2.086213871524472, 0.0000000000000000, -0.7731263170943632, 0.056757289151768725, -0.8068738944278473
  #define MOVE_POSITION_ASSERTION       0.12425244381873693, 0.1932815792735908, 0.006135923151542565, 2.4865828571626243, -0.0966407896367954, -0.44638840927472156, 0.3528155812136975, HEAD_GLOBAL_Y, 0.0015339807878856412, 2.2902333163132624, 0.0000000000000000, 0.20401944478879028, -0.22549517581918926, -0.2991262536377
  #define MOVE_POSITION_OOPS            0.13192234775816514, -1.3330293046726223, 1.217980745581199, 2.1705828148581823, 0.01380582709097077, 1.0, -0.0260776733940559, HEAD_GLOBAL_Y, -0.01687378866674205, 2.801048918679181, 0.0000000000000000, 2.078543967585044, -0.3620194659410113, -0.06289321230331128

  #define MOVE_POSITION_SCRATCH1        0.035281558121369745, -0.2899223689103862, 0.0030679615757712823, 2.12916533358527, 0.044485442848683596, 0.38196121618352463, -0.030679615757712823, HEAD_GLOBAL_Y, -0.0030679615757712823, 0.09817477042468103, 0.0000000000000000, 2.2196702000705226, -0.007669903939428206, -0.1119805975156518
  #define MOVE_POSITION_SCRATCH2        0.02914563496982718, -0.2899223689103862, 0.0015339807878856412, 2.376136240434858, 0.044485442848683596, 0.38196121618352463, -0.030679615757712823, HEAD_GLOBAL_Y, -0.0030679615757712823, 0.09817477042468103, 0.0000000000000000, 2.2396119503130363, -0.007669903939428206, -0.1119805975156518

  #define MOVE_POSITION_HANDSUP1        0.059825250727540004, -0.2500388684253595, 0.01380582709097077, 2.2779614700101773, 0.1487961364249072, -0.056757289151768725, -0.032213596545598466, HEAD_GLOBAL_Y, 0.023009711818284616, 2.2886993355253766, 0.0000000000000000, 0.4724660826687775, -0.4325825821837508, 0.4632621979414636
  #define MOVE_POSITION_HANDSUP2        -0.035281558121369745, -0.0337475773334841, 0.01380582709097077, 0.9740778003073821, 0.10891263593988053, -0.06289321230331128, 0.20708740636456155, HEAD_GLOBAL_Y, 0.035281558121369745, 1.214912784005428, 0.0000000000000000, 1.863786657281054, -0.2638446955163303, 1.9558255045541926                
  #define MOVE_POSITION_HANDSUP3        0.22242721424341796, -0.48627190975974827, 1.2256506495206272, 0.6780195082454534, 1.0231651855197226, -0.1825437137583913, 0.6059224112148283, HEAD_GLOBAL_Y, 0.26691265709210155, 0.461728217153578, 0.0000000000000000, 2.6016314162540475, -0.31139809994078516, 2.9360392280131173

  #define WAVE1                         0.044485442848683596, -0.06135923151542565, 0.0000000000000000, 2.5709518004963345, 0.0000000000000000, -1.3161555160058802, 0.1641359443037636, HEAD_GLOBAL_Y, 0.0000000000000000, 2.0, 0.0000000000000000, 0.4816699673960913, 0.0000000000000000, -0.4000000000000000
  #define WAVE2                         0.04601942363656923, 0.3773592738198677, 0.0000000000000000, 2.4513012990412544, 0.0000000000000000, -1.145883648550574, 0.1641359443037636, HEAD_GLOBAL_Y, 0.0000000000000000, 2.0, 0.0000000000000000, 0.5123495831538042, 0.0000000000000000, -0.4000000000000000
  #define WAVE3                         0.04141748127291231, -0.3620194659410113, 0.0000000000000000, 2.402213913828914, 0.0000000000000000, -1.714990520856147, 0.1641359443037636, HEAD_GLOBAL_Y, 0.0000000000000000, 2.0, 0.0000000000000000, 0.526155410244775, 0.0000000000000000, -0.4000000000000000            

  
  class BehaviorUtils 
  {
    public:
      BehaviorUtils();
      ~BehaviorUtils();


      // NEW STUFF
      void SetHeadPanSpeed(float speed);
      void SetHeadTiltSpeed(float speed);
      void SetHeadSideTiltSpeed(float speed);


      // HEAD CONTROL
      void headHome();
      void moveHead(float sidetilt, float tilt, float pan, float intervalSeconds, bool waitForCompletion);

      // RIGHT ARM CONTROL
      void rightArmHome();
      void moveRightArm(float shoulder_rotate, float shoulder_lift, float elbow_rotate, float elbow_bend, float wrist_rotate, float claw_open, float intervalSeconds, bool waitForCompletion);

      // LEFT ARM CONTROL
      void leftArmHome();
      void moveLeftArm(float shoulder_rotate, float shoulder_lift, float elbow_rotate, float elbow_bend, float wrist_rotate, float claw_open, float intervalSeconds, bool waitForCompletion);

      // WAIST CONTROL
      void bowWaistDown(bool waitForCompletion);
      void bowWaistUp(bool waitForCompletion);
      void bowWaist(bool waitForCompletion);


      private:
        // control_msgs::FollowJointTrajectoryGoal createHeadTrajectoryGoals();

      
      protected:
        ros::NodeHandle nh_;
        //actionlib::SimpleActionServer<behaviorAction> as_;
        //std::string action_name_;
        //behaviorFeedback feedback_;
        //behaviorResult result_;

        // Client actions
        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *head;
        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *rightArm;
        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *leftArm;

        // Subscriptions
        ros::Subscriber waist_calibration_complete_;
        ros::Publisher waist_calibrate_;
        ros::Publisher waist_pos_;

        //ros::Subscriber beam_location;
        //ros::Subscriber speech_keyword;

        // Beam following
        //float last_beam_location;
        //bool keyword_detected;


        ros::ServiceClient head_sidetilt_speed_sc_;
        ros::ServiceClient head_pan_speed_sc_;
        ros::ServiceClient head_tilt_speed_sc_;


    };
};
#endif // __SHELDON_BEHAVIOR_UTILS_H__
