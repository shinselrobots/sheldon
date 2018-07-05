#include <math.h>
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Empty.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/BatteryState.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "boost/lexical_cast.hpp"
#include <assert.h>
//#include <sensor_msgs/Joy.h>
//#include <trajectory_msgs/JointTrajectory.h>
#include <wheel_control/WheelOdomRaw.h>
#include <wheel_control/WheelSpeedRaw.h>

// ROS REP103: "By the right hand rule, the yaw component of orientation increases 
// as the child frame rotates counter-clockwise, 
// and for geographic poses, yaw is zero when pointing east."          


class WheelControl
{
public:
  WheelControl();

private:
  void rawOdomMsgCallback(const wheel_control::WheelOdomRaw::ConstPtr& msg);
  void rawSpeedMsgCallback(const wheel_control::WheelSpeedRaw::ConstPtr& msg);
  void compassMsgCallback(const std_msgs::Float32::ConstPtr& msg);
  void imuOrientationMsgCallback(const geometry_msgs::Point32::ConstPtr& msg);

  void velCmdMsgCallbackP1(const geometry_msgs::Twist::ConstPtr& msg);
  void velCmdMsgCallbackP2(const geometry_msgs::Twist::ConstPtr& msg);
  void velCmdMsgCallbackP3(const geometry_msgs::Twist::ConstPtr& msg);
  void velCmdMsgCallbackP4(const geometry_msgs::Twist::ConstPtr& msg);

  void updateVelocity(int32_t speed_ticks_right, int32_t speed_ticks_left);
  void controlMotors(int32_t speed_ticks_right, int32_t speed_ticks_left);
  void publishOdometry(int32_t odom_ticks_right, int32_t odom_ticks_left);
  void prioritizeCmdMessages(int priority, double speedRequested, double turnRequested);
  double getRotationDegrees(double lastValue, double currentValue);

  ros::NodeHandle nh_;

  ros::Subscriber raw_odom_sub_;
  ros::Subscriber raw_speed_sub_;
  ros::Subscriber compass_sub_;
  ros::Subscriber imu_orientation_sub_;
  ros::Subscriber priority1_sub_;
  ros::Subscriber priority2_sub_;
  ros::Subscriber priority3_sub_;
  ros::Subscriber priority4_sub_;


  tf::TransformBroadcaster odom_broadcaster_;
  ros::Publisher odom_pub_;
  ros::Publisher vel_pub_; // send Motor commands to Sabertooth

  //ros::Timer ramp_timer_;

  // calculated position of robot by odometry in meters
  double x_ = 0.0;
  double y_ = 0.0;
  double w_ = 0.0;                      // rotation/pose of robot
  double w_degrees = 0.0;
  int32_t lastTicksRight_ = 0;
  int32_t lastTicksLeft_ = 0;
  double speedRequested_ = 0.0;         // current speed and turn requested
  double lastSpeedRequested_ = 0.001;   // Make it different than speedRequested to force initializtion
  double turnRequested_ = 0.0;
  double lastTurnRequested_ = 0.001;    // Make it different than turnRequested to force initializtion
  double speedCorrection_ = 0.0;        // correction currently applied to align with requested speed
  double turnCorrection_ = 0.0;         // correction currently applied to align with requested speed
  double currentLinearVelocity_ = 0.0;  // for publishing.  Odom and Speed are received a different rates
  double currentRotationVelocity_ = 0.0; 
  double currentCompassDegrees_ = -0.1;	// compass reported in degrees, so keep that way to avoid rounding errors
  double lastCompassDegrees_ = 0.1;
  double currentImuOrientationDegreesX_ = -1.0;  // IMU reported in degrees, so keep that way to avoid rounding errors
  double lastImuOrientationDegreesX_ = -1.0;
  ros::Time lastCmdMsgTime_, lastRotationVelocityTime_; 
  bool timeOutHandled_ = false;
  int currentPriority_ = 0;
  

  // CONSTANTS
  const bool ROBOT_HAS_COMPASS  = true;
  const bool ROBOT_HAS_IMU  = false; // DAVES TODO!
  const double WHEEL_BASE_METERS = 330.0/1000.0; // mm -> Meters - Tune this value as needed
  const double TICKS_PER_METER = 398.0; // Calibrated, +/- 0.5 ticks!
  const double BASE_CIRMUMFERENCE = M_PI*WHEEL_BASE_METERS;
  const double ANGULAR_SCALE = BASE_CIRMUMFERENCE / (2.0*M_PI); // TODO - CHECK/FIX THIS!
  const double RADIANS_TO_DEGREES = 180.0 / M_PI;
  const double DEGREES_TO_RADIANS = M_PI / 180.0;
  const double PRIORITY_TIMEOUT = 1.0;  // amount of time before lower priority task can take over
  const double DEADMAN_TIMEOUT = 1.0;  // amount of time before motors start to stop automatically

};


WheelControl::WheelControl() //:
  //linear_(1),
  //angular_(0),
  //a_scale_(0.75),
  //l_scale_(1.25)
{
	ROS_INFO("Initializing WheelControl...");

  //nh_.param("axis_linear", linear_, linear_);
  //nh_.param("axis_angular", angular_, angular_);
  //nh_.param("scale_angular", a_scale_, a_scale_);
  //nh_.param("scale_linear", l_scale_, l_scale_);

  // SUBSCRIBERS
  raw_odom_sub_ = nh_.subscribe<wheel_control::WheelOdomRaw>("/wheel_odom_raw", 10, &WheelControl::rawOdomMsgCallback, this);
  raw_speed_sub_ = nh_.subscribe<wheel_control::WheelSpeedRaw>("/wheel_speed_raw", 1, &WheelControl::rawSpeedMsgCallback, this);
  compass_sub_ = nh_.subscribe<std_msgs::Float32>("/compass", 10, &WheelControl::compassMsgCallback, this);
  imu_orientation_sub_ = nh_.subscribe<geometry_msgs::Point32>("/imu_orientation", 10, &WheelControl::imuOrientationMsgCallback, this);

  /// Requested_vel_sub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1, &WheelControl::velCmdMsgCallback, this);
  priority1_sub_ = nh_.subscribe<geometry_msgs::Twist>("move_base/priority1", 1, &WheelControl::velCmdMsgCallbackP1, this);
  priority2_sub_ = nh_.subscribe<geometry_msgs::Twist>("move_base/priority2", 1, &WheelControl::velCmdMsgCallbackP2, this);
  priority3_sub_ = nh_.subscribe<geometry_msgs::Twist>("move_base/priority3", 1, &WheelControl::velCmdMsgCallbackP3, this);
  priority4_sub_ = nh_.subscribe<geometry_msgs::Twist>("move_base/priority4", 1, &WheelControl::velCmdMsgCallbackP4, this);
  
  /// ROS_INFO("WheelControl: subscribed to: /wheel_odom_raw, /wheel_speed_raw, cmd_vel_mux/input/teleop");
  ROS_INFO("WheelControl: subscribed to: /wheel_odom_raw, /wheel_speed_raw");
  ROS_INFO("WheelControl: subscribed to: move_base/priority1 - priority3");


  // PUBLISHERS
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 10, true); // latch last one sent
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/wheel_motors_cmd_vel", 2, true);
	ROS_INFO("WheelControl: publishing topics: odom, /wheel_motors_cmd_vel");

  lastCmdMsgTime_ = ros::Time::now();

}


void WheelControl::rawOdomMsgCallback(const wheel_control::WheelOdomRaw::ConstPtr& msg)
{
	int32_t odomTicksRight = msg->odom_ticks_right;
	int32_t odomTicksLeft = msg->odom_ticks_left;
	// ROS_INFO("WheelControl: Raw: Odom Left=[%d] Right=[%d] ", odomTicksLeft, odomTicksRight );

	// Convert raw values to ROS messages and publish
	publishOdometry(odomTicksRight, odomTicksLeft);
}

void WheelControl::rawSpeedMsgCallback(const wheel_control::WheelSpeedRaw::ConstPtr& msg)
{
	int32_t speedTicksRight = msg->speed_ticks_right;
	int32_t speedTicksLeft = msg->speed_ticks_left;

	/// ROS_INFO("WheelControl: Raw: Speed Left=[%d] Right=[%d]", speedTicksLeft, speedTicksRight);

	// Convert raw values to ROS values for publishing on the odom topic
	updateVelocity(speedTicksRight, speedTicksLeft);

  // Use feedback data to control motor speeds
	controlMotors(speedTicksRight, speedTicksLeft);

}

void WheelControl::compassMsgCallback(const std_msgs::Float32::ConstPtr& msg)
{
	// Keep track of most recent compass reading
	currentCompassDegrees_ = msg->data;
}

void WheelControl::imuOrientationMsgCallback(const geometry_msgs::Point32::ConstPtr& msg)
{
  // Keep track of most recent IMU Orientation (x,y,z) reading
  if(msg->x != currentImuOrientationDegreesX_)
  {
    // DBG-DAVE ROS_INFO_STREAM( "WheelControl: Got IMU update:  Orientation X = " << currentImuOrientationDegreesX_ );
  }

  currentImuOrientationDegreesX_ = msg->x;
  //currentImuOrientationDegreesY_ = msg->y;
  //currentImuOrientationDegreesZ_ = msg->z;
}

// Priority control for motors.  P1 is LOWEST priority!  P0 is idle.
void WheelControl::velCmdMsgCallbackP1(const geometry_msgs::Twist::ConstPtr& msg)
{
  prioritizeCmdMessages( 1, msg->linear.x, msg->angular.z ); // priority, speed, turn
}

void WheelControl::velCmdMsgCallbackP2(const geometry_msgs::Twist::ConstPtr& msg)
{
  prioritizeCmdMessages( 2, msg->linear.x, msg->angular.z ); // priority, speed, turn
}

void WheelControl::velCmdMsgCallbackP3(const geometry_msgs::Twist::ConstPtr& msg)
{
  prioritizeCmdMessages( 3, msg->linear.x, msg->angular.z ); // priority, speed, turn
}

void WheelControl::velCmdMsgCallbackP4(const geometry_msgs::Twist::ConstPtr& msg)
{
  prioritizeCmdMessages( 4, msg->linear.x, msg->angular.z ); // priority, speed, turn
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
void WheelControl::prioritizeCmdMessages(int priority, double speedRequested, double turnRequested)
{
  // Based upon message priority requested, set movement for the base
  // Priority = 0 is idle

  if (priority >= currentPriority_ )
  {
  	// interrupt any lower priority
  	ROS_INFO_STREAM( "WheelControl: Servicing command.  Current Priority = " 
      << currentPriority_ << "  Request Priority = " << priority );

  	currentPriority_ = priority;
  	speedRequested_ = speedRequested; 
  	turnRequested_ = turnRequested; // Positive = Left turn, negative = Right turn
  	lastCmdMsgTime_ = ros::Time::now();
  	timeOutHandled_ = false;
  }
  else
  {
  	ROS_INFO_STREAM( "WheelControl: Ignoring lower priority command.  Current Priority = " 
      << currentPriority_ << "  Request Priority = " << priority );
  }

}


void WheelControl::controlMotors(int32_t speed_ticks_right, int32_t speed_ticks_left)
{
  // A basic active speed control.  Ramps motor speed to desired target, and adjusts for load changes.
  // Calculate current vs. requested speed, and control motors for constant speed
  // This gets called once each time a velocity update arrives (about every 250ms)
  const double FEEDBACK_TICKS_TO_COMMAND_RATIO = 420.0; // ticks for commanded speed of 1.0.  Linear 42 steps per 0.1 twist command.
  const double TURN_TICKS_TO_COMMAND_RATIO = 1.0; // ticks for commanded speed in Radians/Second.
  const double RAMP_INCREMENT = 0.02; //0.01
  const double HYSTERYSIS = 0.04;  // 0.02
  const double FAST_RAMP_HYSTERYSIS = HYSTERYSIS * 4.0; 
  const double MAX_RAMP = 0.2; // 0.1
  const double TURN_HYSTERYSIS = 0.05;   // 0.05
  const double FAST_TURN_HYSTERYSIS = TURN_HYSTERYSIS * 4.0; 
  const double TURN_INCREMENT = 0.02; // 0.01
  const double MAX_TURN_COMPENSATION = 0.2; // 0.1
  const double MAX_SPEED_METERS_PER_SECOND = 3.0; // fastest speed robot is capable of moving (Sabertooth speed = 1.0)
  const double MAX_TURN_RADIANS_PER_SECOND = 8.0; // fastest turn robot is capable of (Sabertooth speed = 1.0)

  double speedCmd = 0.0;
  double turnCmd = 0.0;

  // note: 420 ticks/sec full speed; 42 ticks/sec for each 0.1 step in command speed (pretty linear)
  // TODO!!! make this map to Meters per second and Radians per second!!!


  ros::Duration elapsedTime = ros::Time::now() - lastCmdMsgTime_;

  // Priority Timeout
  if( elapsedTime.toSec() > PRIORITY_TIMEOUT )
  {
  	// Have not received a motor command lately, so allow lower priority behaviors to take control
	currentPriority_ = 0;
  }

  // Deadman Timeout
  if( (elapsedTime.toSec() > DEADMAN_TIMEOUT) && !timeOutHandled_ )
  {
  	// Have not received a motor command lately, stop the motors gradually
  	speedRequested_ = 0.0;
  	turnRequested_ = 0.0;
  	ROS_INFO_STREAM( std::setprecision(2) << std::fixed << 
  	"WheelControl: Command Timeout, stopping motors.  elapsedTime= " << elapsedTime.toSec() );
  	timeOutHandled_ = true;
  }
  else if (!timeOutHandled_)
  {
  	//ROS_INFO_STREAM( std::setprecision(2) << std::fixed << "WheelControl: elapsedTime= " << elapsedTime.toSec() );

  }


  // Trap gross error conditions
  if( (abs(speed_ticks_right) > 1000) || (abs(speed_ticks_right) > 1000)  )
  {
    // Encoder Error!  Should never be greater than about 500 ticks / second!
    ROS_WARN_STREAM( "WHEEL ENCODER ERROR!: IGNORING ENCODERS FOR SPEED CONTROL! speed_ticks_left= " << 
      speed_ticks_left << " speed_ticks_right= " << speed_ticks_right  );

    // Disable velocity control, just pass thorugh what the system requested to the motors
    geometry_msgs::Twist vel;
    vel.linear.x = speedRequested_;
    vel.angular.z = turnRequested_; 
    vel_pub_.publish(vel);  // Send update to the Sabertooth motor controller
    return;
  }


  // Speed Control calculations
   double feedbackRight = (double)speed_ticks_right / FEEDBACK_TICKS_TO_COMMAND_RATIO;
  double feedbackLeft = (double)speed_ticks_left / FEEDBACK_TICKS_TO_COMMAND_RATIO;
  double speedFeedback = (feedbackRight + feedbackLeft) / 2.0;
  double turnFeedback = (feedbackRight - feedbackLeft) / 2.0; 
  if((0.0 != speedRequested_) || (0.0 != turnRequested_)) // || (0.0 != speedFeedback)  || (0.0 != turnFeedback) )
  {
	  ROS_INFO_STREAM( std::setprecision(3) << std::fixed << "WheelControl: TargetSpeed= " << speedRequested_ 
	  	<< " TargetTurn =" << turnRequested_ << " FBSpeed= " << speedFeedback << " FBTurn =" << turnFeedback );
  }

  // SPEED CONTROL

  double speedDelta = speedRequested_ - speedFeedback;

  if( speedDelta > FAST_RAMP_HYSTERYSIS )
  {
  	speedCorrection_ += (RAMP_INCREMENT * 3.0);
  	//ROS_INFO_STREAM(" WheelControl: ++RAMP_INCREMENT, speedCorrection_=" << speedCorrection_);
  }
  else if( speedDelta > HYSTERYSIS )
  {
  	speedCorrection_ += RAMP_INCREMENT;
  	//ROS_INFO_STREAM(" WheelControl: +RAMP_INCREMENT, speedCorrection_=" << speedCorrection_);
  }
  else if( speedDelta < (-FAST_RAMP_HYSTERYSIS) )
  {
  	speedCorrection_ -= (RAMP_INCREMENT * 3.0); 
  	//ROS_INFO_STREAM(" WheelControl: --RAMP_INCREMENT, speedCorrection_=" << speedCorrection_);
  }
  else if( speedDelta < (-HYSTERYSIS) )
  {
  	speedCorrection_ -= RAMP_INCREMENT; 
  	//ROS_INFO_STREAM(" WheelControl: -RAMP_INCREMENT, speedCorrection_=" << speedCorrection_);
  }
  else
  {
  	// close to command requested.  Prevent round-off creep at stop
  	if( 0.0 == speedRequested_ )
  	{
  		speedCorrection_ = 0.0;
	  	//ROS_INFO_STREAM(" WheelControl: SPEED CLAMP AT STOP");
  	}
  }

  // clamp max speedCorrection_
  if(fabs(speedCorrection_) >= MAX_RAMP)
  {
    //ROS_INFO_STREAM("WheelControl:: DBG: Exceeds MAX_RAMP: speedCorrection_ = " << speedCorrection_);
    if(speedCorrection_ > 0)
    {
      speedCorrection_ = MAX_RAMP;
      //ROS_INFO_STREAM(" WheelControl: Clamping to MAX_RAMP");
    }
    else if(speedCorrection_ < 0)
    {
      speedCorrection_ = -MAX_RAMP;
      //ROS_INFO_STREAM(" WheelControl: Clamping to Negative MAX_RAMP");
    }

  }
  speedCmd = speedRequested_ + speedCorrection_;



  // TURN CONTROL
  double turnDelta = turnRequested_ - turnFeedback;

  if( (turnRequested_ != lastTurnRequested_) || (speedRequested_ != lastSpeedRequested_) )
  {
    // New turn command
    turnCmd = turnRequested_; // immediately apply requested turn
  }
  else
  {

    double turnDelta = turnRequested_ - turnFeedback;

    if( turnDelta > FAST_TURN_HYSTERYSIS )
    {
      turnCorrection_ += (TURN_INCREMENT * 2.0);
      //ROS_INFO_STREAM(" WheelControl: ++TURN_INCREMENT");
    }
    else if( turnDelta > TURN_HYSTERYSIS )
    {
      turnCorrection_ += TURN_INCREMENT;
      //ROS_INFO_STREAM(" WheelControl: +TURN_INCREMENT");
    }
    else if( turnDelta < -FAST_TURN_HYSTERYSIS )
    {
      turnCorrection_ -= (TURN_INCREMENT * 2.0); 
      //ROS_INFO_STREAM(" WheelControl: --TURN_INCREMENT");
    }
    else if( turnDelta < -TURN_HYSTERYSIS )
    {
      turnCorrection_ -= TURN_INCREMENT; 
      //ROS_INFO_STREAM(" WheelControl: -TURN_INCREMENT");
    }
    else
    {
      // close to command requested.  Prevent round-off creep at center
      if( 0.0 == turnRequested_ )
      {
        turnCorrection_ = 0.0;
        //ROS_INFO_STREAM(" WheelControl: TURN CLAMP AT CENTER");
      }
    }

    // clamp max turnCorrection_
    if(fabs(turnCorrection_) >= MAX_RAMP)
    {
      if(turnCorrection_ > 0)
      {
        turnCorrection_ = MAX_RAMP;
        //ROS_INFO_STREAM(" WheelControl: Clamping TURN to MAX_RAMP");
      }
      else if(turnCorrection_ < 0)
      {
        turnCorrection_ = -MAX_RAMP;
        //ROS_INFO_STREAM(" WheelControl: Clamping TURN to Negative MAX_RAMP");
      }

    }
    turnCmd = turnRequested_ + turnCorrection_;

  }

 
  if((0.0 != speedRequested_) || (0.0 != speedFeedback) || (0.0 != speedDelta) || (0.0 != speedCmd) ||
     (0.0 != turnRequested_)  || (0.0 != turnFeedback)  || (0.0 != turnDelta) || (0.0 != turnCmd))
  {
  	//* DBG-DAVE
    ROS_INFO_STREAM( std::setprecision(3) << std::fixed << 
      "WheelControl: SPEED: Target=" << speedRequested_ << " Feedback=" << speedFeedback <<
      " Delta=" << speedDelta << " Bias=" << speedCorrection_ << " Cmd=" << speedCmd );

    ROS_INFO_STREAM( std::setprecision(3) << std::fixed << 
      "WheelControl: TURN:  Target=" << turnRequested_ << " Feedback=" << turnFeedback <<
      " Delta=" << turnDelta << " Bias=" << turnCorrection_ <<" Cmd=" << turnCmd );
    //*/
    // NOTE!  If motor does not run, check that voltate to Sabertooth is not too high (plugged into charger)
  }

  lastTurnRequested_ = turnRequested_;
  lastSpeedRequested_ = speedRequested_;

  geometry_msgs::Twist vel;
  vel.linear.x = speedCmd / MAX_SPEED_METERS_PER_SECOND; // NOTE: Sabertooth value of +/- 1.0 is full speed!
  vel.angular.z = turnCmd / MAX_TURN_RADIANS_PER_SECOND; // NOTE: Sabertooth value of +/- 1.0 is full speed!
  vel_pub_.publish(vel); // Send update to the Sabertooth motor controller

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
void WheelControl::updateVelocity(int32_t speed_ticks_right, int32_t speed_ticks_left)
{
  // Arduino publishes ticks/second. Convert to ROS standard units (meters, m/s)
  // Sets currentLinearVelocity_ and currentRotationVelocity_ for publishOdometry() which is called more often

  // ROS_INFO("---SPEED---"); // separate frames of data on output
  double speedLeft = speed_ticks_left / TICKS_PER_METER;
  double speedRight = speed_ticks_right / TICKS_PER_METER; // convert ticks/sec to meter/sec
  

  currentLinearVelocity_ =  (speedLeft + speedRight) / 2.0; // Linear Speed

  if( ROBOT_HAS_IMU || ROBOT_HAS_COMPASS )
  {
  	// rotation velocity is calculated in PublishOdometry(), 
  	// using IMU or compass and elapsed time instead of wheel speeds
  }
  else
  {
  	double wheelSpeedDifference =  (speedLeft - speedRight) / 2.0; // Angular Speed
  	currentRotationVelocity_ = wheelSpeedDifference / ANGULAR_SCALE;
  }


  // Print velocity if it is non-zero, for debug purposes

  /* DBG-DAVE
    if (fabs(currentLinearVelocity_) + fabs(currentRotationVelocity_) > 0) {
    ROS_INFO_STREAM(std::setprecision(3) << std::fixed <<
    "WheelControl: updateVelocity: Ticks/sec=[ " << speed_ticks_left << ", " << speed_ticks_right << 
    " ] Meters/sec: Linear= " << currentLinearVelocity_ << " Angular= " << currentRotationVelocity_);
  }
  */
  
}

double WheelControl::getRotationDegrees(double lastValue, double currentValue)
{
  // calculate the shortest rotation between two vectors in degrees
  // Assume the rotation did not go the long way around the circle!
  
      double rotationAmt = currentValue - lastValue;
      if(rotationAmt > 180)
      {
          rotationAmt -=360;
      }
      else if(rotationAmt < -180)
      {
          rotationAmt +=360;
      }

      if(0.0 != rotationAmt)
      {
        // ROS_INFO_STREAM( "Rotation Update: last= " << lastValue << " cur= " << currentValue << " Amt= " << rotationAmt << " deg, " << (rotationAmt * DEGREES_TO_RADIANS) << " rad" );
      }
      return rotationAmt;
}

void WheelControl::publishOdometry(int32_t odom_ticks_right, int32_t odom_ticks_left)
{
  // ROS_INFO("---ODOM---"); // separate frames of data on output
  // Note: Arduino publishes raw ticks to avoid rounding errors (does not have double precision)

  // get distance traveled (in ticks) since last update
  int32_t ticksUpdateRight = odom_ticks_right - lastTicksRight_; 
  lastTicksRight_ = odom_ticks_right;
  int32_t ticksUpdateLeft = odom_ticks_left - lastTicksLeft_;
  lastTicksLeft_ = odom_ticks_left;
  /// ROS_INFO_STREAM( "WheelControl: Update: ticksUpdateLeft= " << ticksUpdateLeft << " ticksUpdateRight= " << ticksUpdateRight );

  // convert raw odometry data to ROS standard units (meters, m/s)
  double odomRight = odom_ticks_right / TICKS_PER_METER;
  double odomLeft = odom_ticks_left / TICKS_PER_METER;
  /// ROS_INFO_STREAM( "WheelControl: Meters: (odomLeft= " << odomLeft << " odomRight= " << odomRight );

  // Transform distance traveled by each wheel into linear and angular values in meters and radians
  double updateRight = ticksUpdateRight / TICKS_PER_METER;
  double updateLeft = ticksUpdateLeft / TICKS_PER_METER;
  double distanceUpdate =  (updateLeft + updateRight) / 2.0;  // Linear Distance
  double rotationUpdateRadians = 0.0;
  double rotationUpdateDegrees = 0.0;

  if( ROBOT_HAS_IMU )
  {
  
    if(currentImuOrientationDegreesX_ < 0.0) 
    {
      // uninitilized.  Wait for first reading
      currentRotationVelocity_ = 0.0;  
      ROS_WARN_STREAM( "ROBOT_HAS_IMU: waiting for first reading " );
    }
    else if(lastImuOrientationDegreesX_ < 0.0)
    {
      // first update
      currentRotationVelocity_ = 0.0;  
      lastImuOrientationDegreesX_ = currentImuOrientationDegreesX_;
      lastRotationVelocityTime_ = ros::Time::now();
      ROS_INFO_STREAM( "ROBOT_HAS_IMU: recieved first reading " );
    }
    else
    {
      // calculate rotation AMOUNT (according to the IMU) since last update

      rotationUpdateDegrees = getRotationDegrees(lastImuOrientationDegreesX_, currentImuOrientationDegreesX_);
      lastImuOrientationDegreesX_ = currentImuOrientationDegreesX_;
      rotationUpdateRadians = rotationUpdateDegrees * DEGREES_TO_RADIANS;

      // calculate rotation VELOCITY using the IMU, in radians per second
      // Time since last velocity calculation
      ros::Time currentTime = ros::Time::now();
      ros::Duration elapsedTime = currentTime - lastRotationVelocityTime_;
      lastRotationVelocityTime_ = currentTime;
      currentRotationVelocity_ = rotationUpdateRadians / elapsedTime.toSec();
      if(0.0 != rotationUpdateRadians)
      {
      	ROS_INFO_STREAM( "ROBOT_HAS_IMU:  rotation update = " << rotationUpdateRadians << " Radians/sec, " <<  rotationUpdateDegrees << " Degrees/sec");
      }
    }

  }
  else if( ROBOT_HAS_COMPASS )
  {

    //ROS_INFO_STREAM( "ROBOT_HAS_COMPASS:  currentCompassDegrees_ = " << currentCompassDegrees_);

    if(currentCompassDegrees_ < 0.0) 
    {
      // uninitilized.  Wait for first reading
      currentRotationVelocity_ = 0.0;  
      ROS_WARN_STREAM( "ROBOT_HAS_COMPASS: waiting for first reading " );
    }
    else if(lastCompassDegrees_ < 0.0)
    {
      // first update
      currentRotationVelocity_ = 0.0;  
      lastCompassDegrees_ = currentCompassDegrees_;
      lastRotationVelocityTime_ = ros::Time::now();
      ROS_INFO_STREAM( "ROBOT_HAS_COMPASS: received first reading " );
    }
    else
    {

    	// calculate rotation AMOUNT (according to the compass) since last update, in radians
      rotationUpdateDegrees = getRotationDegrees(lastCompassDegrees_, currentCompassDegrees_);
      lastCompassDegrees_ = currentCompassDegrees_;
      rotationUpdateRadians = rotationUpdateDegrees * DEGREES_TO_RADIANS;

   	  // calculate rotation VELOCITY using the compass, in radians per second
      // Time since last velocity calculation
      ros::Time currentTime = ros::Time::now();
      ros::Duration elapsedTime = currentTime - lastRotationVelocityTime_;
      lastRotationVelocityTime_ = currentTime;
      currentRotationVelocity_ = rotationUpdateRadians / elapsedTime.toSec();
      //ROS_INFO_STREAM( "ROBOT_HAS_COMPASS:  rotation update = " << rotationUpdateRadians << " Radians");
    }
  }
  else
  {
    // NOTE: if no compass, rotation velocity is calculated by difference of wheel speeds in updateVelocity()
    // So we just calculate rotation amount here.
	  double difference =  (updateLeft - updateRight) / 2.0;      // for Angular Distance
  	rotationUpdateRadians = difference / ANGULAR_SCALE;
    if(0.0 != difference)
    {
      ROS_INFO_STREAM( "ROBOT_NO_COMPASS:  rotation update = " << rotationUpdateRadians << " Radians");
    }
  }


  // Integrate odometry
  //   Differential drive, so vy always zero in local frame.
  //   However, distance needs to be rotated by current orientation to 
  //   map into odometry frame.
  double dx = cos(w_)*distanceUpdate;
  double dy = sin(w_)*distanceUpdate;
  double dw = rotationUpdateRadians;
  x_ += dx;
  y_ += dy;
  w_ += dw;
  w_degrees += rotationUpdateDegrees;

  ///ROS_INFO_STREAM( "WheelControl: Pose Update Calc: w = " << w_ << " dx = " << dx << " dy= " << dy );

  // Print updated odom pose (if it changes), for debug purposes
  if (fabs(dx) + fabs(dw) > 0) {
    //double staticRotationDegrees = w_ * RADIANS_TO_DEGREES; // 180.0 / M_PI
    ////ROS_INFO( "PUB ODOM: Static Pose x= %2.2f, y= %2.2f, w= %2.4f rad, %2.2f deg", x_, y_, w_, (w_ * RADIANS_TO_DEGREES) );
    //ROS_INFO( "DBG_deg:  Update= %2.4f, Pose w= %2.4f degrees, %2.4f radians", rotationUpdateDegrees, w_degrees, (w_degrees * DEGREES_TO_RADIANS) );
    //ROS_INFO_STREAM( "WheelControl: Static Pose (x= " << x_ << " y= " << y_ << " w_= " << w_ << ") w_ deg = " << staticRotationDegrees << "w_degrees= " << w_degrees );
  }

  // Format data for Publishing
  // All odometry is 6DOF so need a quaternion created from yaw
  geometry_msgs::Quaternion odom_quat =  tf::createQuaternionMsgFromYaw(w_);

  // Create an odometry message 
  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = "odom";
  //odom.child_frame_id = "base_footprint";

  // Set the accumulated pose as a 3D (6DOF) value
  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  // Set the velocity as a twist relative to the child frame
  // Velocity comes from updateVelocity(), updated less often then Odom
  odom.twist.twist.linear.x = currentLinearVelocity_; 
  odom.twist.twist.linear.y = 0;
  odom.twist.twist.angular.z = currentRotationVelocity_; 

  // Publish odom message 
  odom_pub_.publish(odom);

  // Echo header data from odom message to odom transform header
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = odom.header.stamp; 
  odom_trans.header.frame_id = odom.header.frame_id; 
  odom_trans.child_frame_id = "base_footprint";

  // Echo pose data from odom message to odom transform
  odom_trans.transform.translation.x = odom.pose.pose.position.x;
  odom_trans.transform.translation.y = odom.pose.pose.position.y;
  odom_trans.transform.translation.z = odom.pose.pose.position.z;
  odom_trans.transform.rotation = odom.pose.pose.orientation;

  // Broadcast the odom transform  
  odom_broadcaster_.sendTransform(odom_trans);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////


int main(int argc, char** argv)
{
  ros::init(argc, argv, "wheel_control");
  WheelControl wheel_control;
  ros::spin();
}
