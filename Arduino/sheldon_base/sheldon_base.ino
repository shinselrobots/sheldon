#include <Wire.h>                  // for I2C
#include <Encoder.h>               // for reading Quadrature Encoders (waist)
#include <MeetAndroid.h>           // for bluethooth control from Android Phone
#include <Sabertooth.h>
//#include <Adafruit_ADS1015.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// ROS
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/Joy.h>
#include <ros/time.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/BatteryState.h>
#include <behavior_common/CommandState.h>

#include "RobotConstants.h"

// Constants
#define PI                          3.1415926535
#define WAIST_TICKS_MAX             106.0  // number of ticks leaning forward from calibrated "zero" point (standing straight up)
#define WAIST_DEGREES_MAX           58.0

// DEBUG ONLY: Enable for debug to control waist from BT Phone, keys 9,10,11.
const boolean BT_WAIST_DEBUG = 0;  

const float WAIST_TICKS_TO_RADIANS    = (PI * WAIST_DEGREES_MAX) / (180 * WAIST_TICKS_MAX); // calculate at comple time for efficiency
const float WAIST_RADIANS_TO_TICKS    = (180 * WAIST_TICKS_MAX) / (PI * WAIST_DEGREES_MAX);
const int   WAIST_TICKS_HOME          = 1; // was 3: number of ticks from calibrated "zero" point to home (standing straight up)

const float WAIST_POSITION_HYSTERESIS = 4;   // Ticks... about 2 degrees
const int WAIST_MOTOR_SPEED_STOP      = 0;
const int WAIST_MOTOR_SPEED_SLOW      = 32;       // Max Sabertooth speed is +/- 127
const int WAIST_MOTOR_SPEED_MED       = 64;       // Max Sabertooth speed is +/- 127
const int WAIST_MOTOR_SPEED_FAST      = 96;       // Max Sabertooth speed is +/- 127
const int WAIST_MOTOR_SPEED_MAX       = 127;       // Max Sabertooth speed is +/- 127
const int DEBUG_STRING_LEN            = 80;
const int NUMBER_OF_JOYSTICK_AXIS     = 12;
const int NUMBER_OF_JOYSTICK_BUTTONS  = 12;

enum WAIST_CALIBRATE_STATE_T {
  WAIST_NOT_CALIBRATED = 0,
  WAIST_CAL_BEGIN,
  WAIST_CAL_MOVE_DOWN_WAIT,
  WAIST_CAL_MOVE_UP,
  WAIST_CAL_LOOK_FOR_LIMIT_SWITCH,
  WAIST_CAL_MOVING_TO_HOME,
  WAIST_CAL_COMPLETED
};


// ROS Messages

/////////////////////////////////////////////////////////////////////////////////////
// Global Variables

MeetAndroid       meetAndroid; // For Android Bluetooth connection
boolean           PowerIsOn = false; // Keeps board from running on USB power; requres external power to be turned on
ARDUINO_STATUS_T  Status;   // Status and Command buffers for communicating with the PC
ANDROID_STATUS_T  Android;
int               lastAndroidCmd;
Adafruit_BNO055 bno = Adafruit_BNO055();

WAIST_CALIBRATE_STATE_T waistCalibrateState = WAIST_NOT_CALIBRATED;

int               targetWaistPosition = 0;  // Ticks from *cal* position
int               currentWaistPosition = 0; // Ticks from *cal* position
int               newWaistMotorCommand = 0;
int               lastWaistMotorCommand = 0;
//int               waistRampSpeed = 0;
boolean           waistLimitUp = false;
boolean           waistLimitDown = false;
boolean           lastWaistLimitUp = false;
boolean           lastWaistLimitDown = false;
boolean           waistRosMoveInProgress = false;
unsigned long     waistCalStartTime = 0;

String            debugString;
char              debugStringChar[80];
float             JoyStickAxis[NUMBER_OF_JOYSTICK_AXIS];        // Array of Axis the Joystick can set
long              JoyStickButtons[NUMBER_OF_JOYSTICK_BUTTONS];  // Array of Buttons the Joystick can set


// Joint State Message
sensor_msgs::JointState waist_joint_states_msg;
                  //creating the arrays for the message
char              *name_array[] = {"knee_joint", "hip_joint"};
float             position_array[]={0,0};
float             velocity_array[]={0,0};
float             effort_array[]={0,0};

ros::NodeHandle   nh;


/////////////////////////////////////////////////////////////////////////////////////
// SUBSCRIBERS
void waist_calibrate_cmd_callback(const std_msgs::Empty& cmd_msg) {
  nh.loginfo("ARDUINO: Got WAIST COMMAND CALIBRATE message");
  waistCalibrateState = WAIST_CAL_BEGIN;
}
ros::Subscriber<std_msgs::Empty> wasteCalibrateSubscriber("\waist_calibrate", &waist_calibrate_cmd_callback); // does this need & ?

void waist_position_cmd_callback(const std_msgs::Float32& cmd_msg) {
  // waist position is in radians from *Home*.  must convert to Ticks from *cal*.
  if (WAIST_CAL_COMPLETED == waistCalibrateState) {
    nh.loginfo("ARDUINO: Got WAIST COMMAND message");
    targetWaistPosition = (int)(((float)cmd_msg.data * WAIST_RADIANS_TO_TICKS) + 0.5) + WAIST_TICKS_HOME; // 0.5 to avoid roundoff error
//    waistRampSpeed = 0;
    waistRosMoveInProgress = true;  // allow ramp controller to take control
    debugString = String("DEBUG: Waist Target Ticks = ") + String(targetWaistPosition);
    debugString.toCharArray(debugStringChar, DEBUG_STRING_LEN );
    nh.loginfo(debugStringChar);
  }
  else {
    nh.loginfo("ARDUINO: WAIST COMMAND message IGNORED!  Not Calibrated!");
  }
}
ros::Subscriber<std_msgs::Float32> waistPositionSubscriber("\waist_goal_position", &waist_position_cmd_callback); // does this need & ?


// Sabertooth motor control for Waist (knee and hip joints mechanically connected)
Sabertooth ST(128, Serial2); // Address 128 (default), but use Serial2 as the serial port.

// Assign pins for quadrature encoder.
// Mega pins with interrupt are: 2,3, plus 18,19,20,21 (reserved for Serial1 and I2C)
Encoder waistEncoder(KNEE_ENCODER_A_PIN, KNEE_ENCODER_B_PIN);


/////////////////////////////////////////////////////////////////////////////////////
// PUBLISHERS
std_msgs::Float32 compassMsg; // from Devantech CMPS11 Compass
ros::Publisher pub_compass("/compass", &compassMsg);

geometry_msgs::Point32 imuOrientationMsg;  // from Adafruit BNO055 IMU (assumes BNO055 compass not calibrated)
ros::Publisher pub_imu_orientation("/imu_orientation", &imuOrientationMsg); // direction robot is facing in x,y,z

std_msgs::Int16 imuTemperatureMsg;  // from Adafruit BNO055 IMU
ros::Publisher pub_imu_temperature("/ambient_temperature", &imuTemperatureMsg); // in Celsius

// Waist feedback is provided as knee and hip joints for accurate URDF modeling
//std_msgs::Float32 kneePositionMsg;
//ros::Publisher pub_knee_position("/knee_current_position", &kneePositionMsg);
//std_msgs::Float32 hipPositionMsg;
//ros::Publisher pub_hip_position("/hip_current_position", &hipPositionMsg);
ros::Publisher pub_waist_joint_states("/joint_states", &waist_joint_states_msg);

sensor_msgs::Joy bt_motor_cmd_msg;
ros::Publisher pub_bluetooth_motor_cmd("/joy", &bt_motor_cmd_msg);

behavior_common::CommandState behavior_cmd_msg;
ros::Publisher pub_behavior_cmd("/behavior/cmd", &behavior_cmd_msg);

/////////////////////////////////////////////////////////////////////////////////////
// UTILITY FUNCTIONS
void sendWaistMotorCommand(int newCommand)
{
  if ( newCommand != lastWaistMotorCommand ) {
    debugString = String("ARDUINO DBG: Sending waist motor command: ") + String(newCommand);
    debugString.toCharArray(debugStringChar, DEBUG_STRING_LEN );
    nh.loginfo(debugStringChar);

    //compensate for motor running backward (should change wires at some point)
    ST.motor(1, -newWaistMotorCommand); // Motor number, speed/direction ( -127 to 127)
    lastWaistMotorCommand = newCommand;
  }
  else {
    //debugString = String("DEBUG: Ignoring duplicate waist motor command: ") + String(newCommand);
    //debugString.toCharArray(debugStringChar, DEBUG_STRING_LEN );
    //nh.loginfo(debugStringChar);
  }

}


/////////////////////////////////////////////////////////////////////////////////////
//                            SETUP
/////////////////////////////////////////////////////////////////////////////////////
void setup(void)  {

  for ( int i = 0; i < 3; i++ )  {
    AllLedsOn();
    delay(10);
    AllLedsOff();
    delay(50);
  }
  lastAndroidCmd  = -1;

  // Initialize ROS
  nh.initNode();

  // Serial.begin(19200);   // NOTE: USB Serial port communication with the PC handled by ROS node library
  Serial1.begin(115200);    // bluetooth connection with Android phone
  Serial2.begin(9600);      // Sabertooth motor control for Waists
  if(bno.begin()) {         // Will automatically start I2C for all devices
    delay(1000);
    bno.setExtCrystalUse(true);
  }
  else {
    nh.logerror("ARDUINO: CAN'T FIND BNO055 IMU!");
    bno = NULL;
  }
  // Wire.begin(); // Don't use, since BNO055 library calls "Wire.begin()" internally
  
  nh.loginfo("ARDUINO MEGA: ROS Init Start");
  nh.subscribe(wasteCalibrateSubscriber);
  nh.subscribe(waistPositionSubscriber);
  nh.advertise(pub_compass);
  nh.advertise(pub_imu_orientation);
  nh.advertise(pub_imu_temperature);

  //nh.advertise(pub_knee_position);
  //nh.advertise(pub_hip_position);
  nh.advertise(pub_waist_joint_states);
  
  nh.advertise(pub_bluetooth_motor_cmd);
  nh.advertise(pub_behavior_cmd);
  // while(!nh.connected()) nh.spinOnce();
  //  nh.loginfo("Arduino ROS connected");

  
  //nh.loginfo("Attaching Interrupts");
  //attachInterrupt(INT_0, CounterISR0, CHANGE);
  //attachInterrupt(INT_1, CounterISR1, CHANGE);

  pinMode(HEARTBEAT_LED_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(KNEE_ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(KNEE_ENCODER_B_PIN, INPUT_PULLUP);
  pinMode(KNEE_LIMIT_UP_PIN, INPUT_PULLUP);
  pinMode(KNEE_LIMIT_DOWN_PIN, INPUT_PULLUP);

  // Setup Relay contol pins
  //pinMode(RELAY_BOARD_PIN_0, OUTPUT);
  //pinMode(AUX_LIGHT_PIN, OUTPUT);
  //pinMode(SERVO_PWR_18V_PIN, OUTPUT);
  //pinMode(SERVO_PWR_12V_PIN, OUTPUT);
  //digitalWrite(RELAY_BOARD_PIN_0, false);  // N/C
  //digitalWrite(AUX_LIGHT_PIN, false);      // Blue lights off by default
  //digitalWrite(SERVO_PWR_18V_PIN, true);   // Servo Power ON by default, FOR NOW!
  //digitalWrite(SERVO_PWR_12V_PIN, true);


  // Bluetooth using Amarilo - register callback functions for Bluetooth
  meetAndroid.registerFunction(ConnectionEvent, 'C');
  meetAndroid.registerFunction(EnableAccelerometerEvent, 'E');
  meetAndroid.registerFunction(eXecuteCmdEvent, 'X');
  meetAndroid.registerFunction(AccelerometerEvent, 'A');


  // Fast access for general I/O pins.  Set Ports A and C as inputs
  // Port A = Pins 22-29.  Port C = Pins 37 - 30 (reverse order)
  // Port C hardware bumpers are pulled low when activated
  // DDRD = DDRD | B11111100; // example for setting selected pins only
  // PORTx = B10101000; // sets digital pins 7,5,3 HIGH.  PORTx can be used for read or write
  // PINx // Read port at once. read only.
  //DDRA = 0x00; // same as B00000000;  // sets Arduino Port A [pins7...0], 1=output, 0=input
  //DDRC = 0x00; // same as B00000000;  // sets Arduino Port A [pins7...0], 1=output, 0=input
  //PORTA = 0xFF; // Set pullup resistors on to avoid noise on unused pins
  //PORTC = 0xFF; // Set pullup resistors on to avoid noise on unused pins

  // Bluetooth phone as a Joystick
  bt_motor_cmd_msg.axes = JoyStickAxis;
  bt_motor_cmd_msg.axes_length = NUMBER_OF_JOYSTICK_AXIS;
  bt_motor_cmd_msg.axes[0] = 0.0;
  bt_motor_cmd_msg.axes[1] = 0.0;

  bt_motor_cmd_msg.buttons = JoyStickButtons;
  bt_motor_cmd_msg.buttons_length = NUMBER_OF_JOYSTICK_BUTTONS;
  for ( int i=0; i < NUMBER_OF_JOYSTICK_BUTTONS; i++ ) {
    bt_motor_cmd_msg.buttons[i] = 0;
  }

  // Waist Joints (knee and hip)
  //kneePositionMsg.data = 0.0;
  //hipPositionMsg.data = 0.0;

  // Initialize and assign the arrays to the message
  waist_joint_states_msg.header.frame_id = "base_link";
  // waist_joint_states_msg.header.stamp = nh.now();
  waist_joint_states_msg.name = name_array;
  waist_joint_states_msg.position = position_array;
  waist_joint_states_msg.velocity = velocity_array;
  waist_joint_states_msg.effort = effort_array;
  
  //set the array length (number of joints)
  waist_joint_states_msg.name_length = 2;
  waist_joint_states_msg.position_length = 2;
  waist_joint_states_msg.velocity_length = 2;
  waist_joint_states_msg.effort_length = 2;

  nh.loginfo("ARDUINO Init Complete");
  // Blink the lights to show the board is booting up
  for ( int i = 0; i < 10; i++ )
  {
    AllLedsOn();
    delay(10);
    AllLedsOff();
    delay(50);
  }

} // End of setup


/////////////////////////////////////////////////////////////////////////////////////
//                            LOOP
/////////////////////////////////////////////////////////////////////////////////////
void loop(void)
{

  //static unsigned long LastLoopTime = 0;
  //unsigned long LoopStartTime = millis();
  AllLedsOff();  // Turn off LEDs (they just blink on for 20ms)

  // Read fast IO sensors (bumpers/switches), so we don't miss transients
  // for now,we just read these, but in the future should read the port at once, as shown in ReadFastDigitalPorts();

  //////////////////////////////////////////////////////////////
  // Waist Limit Switches
  waistLimitUp = digitalRead(KNEE_LIMIT_UP_PIN);  // Active High, switches are Normally Closed to ground
  if ( waistLimitUp != lastWaistLimitUp) {
    // state changed
    if (waistLimitUp) {
      nh.loginfo("ARDUINO: Waist Up Limit!");
      // Reached maximum throw of the actuator.
      //newWaistMotorCommand = WAIST_MOTOR_SPEED_STOP;
    }
    lastWaistLimitUp = waistLimitUp;
  }

  waistLimitDown = digitalRead(KNEE_LIMIT_DOWN_PIN);  // Active High
  if ( waistLimitDown != lastWaistLimitDown) {
    if (waistLimitDown) {
      nh.loginfo("ARDUINO: Waist Down Limit!");
      // Reached maximum throw of the actuator.
      //newWaistMotorCommand = WAIST_MOTOR_SPEED_STOP;
    }
    lastWaistLimitDown = waistLimitDown;
  }


  //////////////////////////////////////////////////////////////
  // Waist Calibration State Machine
  switch (waistCalibrateState)
  {
    // Behavior modes - what to do while standing around
    // Some of these are mutually exclusive!

    case WAIST_NOT_CALIBRATED: // wait for user to request that we do the calibration moves
    case WAIST_CAL_COMPLETED:  // done, no further acton needed
      {
        break;
      }
    case WAIST_CAL_BEGIN:
      {
        // Actuators will disable if at end of travel, so force move down/forward to start
        newWaistMotorCommand = WAIST_MOTOR_SPEED_SLOW; // move down/forward
        waistCalStartTime = millis();
        waistCalibrateState = WAIST_CAL_MOVE_DOWN_WAIT;
        break;
      }
    case WAIST_CAL_MOVE_DOWN_WAIT:
      {
        // move to down while waiting for timer to expire. can't rely on the encoder yet.
        unsigned long CurrentTime = millis();
        if ( (CurrentTime - waistCalStartTime) > 1000 )  { //milliseconds
          // Time's up, go to next step
          nh.loginfo("ARDUINO: WAIST_CAL_MOVE_DOWN_WAIT timer completed, going to next step");
          newWaistMotorCommand = WAIST_MOTOR_SPEED_STOP;
          waistCalibrateState = WAIST_CAL_MOVE_UP;
        }
        break;
      }
    case WAIST_CAL_MOVE_UP:
      {
        // move to full upright state, while monitoring limit switch
        newWaistMotorCommand = -WAIST_MOTOR_SPEED_SLOW; // move up/back
        waistCalibrateState = WAIST_CAL_LOOK_FOR_LIMIT_SWITCH;
        break;
      }
    case WAIST_CAL_LOOK_FOR_LIMIT_SWITCH:
      {
        // move to full upright state, while monitoring limit switch
        if (waistLimitUp) {
          // limit switch triggered. Save top of range.
          currentWaistPosition = 0;  // Set top of range at zero.
          waistEncoder.write(0);
          nh.loginfo("ARDUINO: Waist Encoder Calibrated! <=========");
          // now, move to home position (standing straight up)
          targetWaistPosition = 0;
          newWaistMotorCommand = WAIST_MOTOR_SPEED_SLOW; // move forward/down
          waistCalibrateState = WAIST_CAL_MOVING_TO_HOME;
        }
        else {
          /// DEBUG:  nh.loginfo("Waist Cal: MOVE UP Looking for Limit switch");
        }
        break;
      }
    case WAIST_CAL_MOVING_TO_HOME:
      {
        // handled in encoder
        break;
      }

    default:
      {
        debugString = String("ARDUINO: ERROR! Bad Calibration State = ") + String(waistCalibrateState);
        debugString.toCharArray(debugStringChar, DEBUG_STRING_LEN );
        nh.loginfo(debugStringChar);
      }

  }


  //////////////////////////////////////////////////////////////
  // Waist Position Encoder
  // waist position commands are in Ticks from *cal* position.  Positive = lean forward.
  long newEncoderValue = waistEncoder.read();

  // first, see if we are done calibrating
  if (WAIST_CAL_MOVING_TO_HOME == waistCalibrateState) {
    // currently moving to home state
    if (newEncoderValue >= WAIST_TICKS_HOME)
    {
      // reached home
      newWaistMotorCommand = WAIST_MOTOR_SPEED_STOP;
      waistCalibrateState = WAIST_CAL_COMPLETED;
      nh.loginfo("ARDUINO: ******* WAIST CALIBRATION COMPLETED *******");
    }
  }

  if (WAIST_CAL_COMPLETED == waistCalibrateState) {
    // update encoder values after it has been calibrated
    if (newEncoderValue != currentWaistPosition) {

      // publish current position in Radians
      // calculation from measurements, see "sheldon_motor_calculations.xlsx"
      //kneePositionMsg.data = ((float)(newEncoderValue - WAIST_TICKS_HOME) * 0.00770) - 0.0070;
      //hipPositionMsg.data =  ((float)(newEncoderValue - WAIST_TICKS_HOME) * 0.01740) - 0.0137;

      position_array[0] = ((float)(newEncoderValue - WAIST_TICKS_HOME) * 0.00770) - 0.0070;
      position_array[1] =  ((float)(newEncoderValue - WAIST_TICKS_HOME) * 0.01740) - 0.0137;   

      //if(abs(newEncoderValue-currentWaistPosition) > 1) { // ignore small motions due to vibration
        debugString = String("ARDUINO: DBG: Encoder Ticks = ") + String(newEncoderValue - WAIST_TICKS_HOME) + 
          String(" Knee Rad: ") + String(position_array[0]) + String(" Hip Rad: ") + String(position_array[1]);
        debugString.toCharArray(debugStringChar, DEBUG_STRING_LEN );
        nh.loginfo(debugStringChar);
      //}
      currentWaistPosition = newEncoderValue;
    }
  }


  //////////////////////////////////////////////////////////////
  // Handle Waist Move Command
  // move to target waist position if we are not already there
  if ((WAIST_CAL_COMPLETED == waistCalibrateState) && (waistRosMoveInProgress)) {
    float waistPositionDelta = targetWaistPosition - currentWaistPosition;
    float waistMoveSpeed = waistPositionDelta * 4.0; // TODO TUNE THIS

    if ( (waistPositionDelta > 0) && (waistPositionDelta > WAIST_POSITION_HYSTERESIS) )  {
      // Move forward/down
      nh.loginfo("DEBUG: Waist MOVE FORWARD!");
      debugString = String("Waist MOVE FORWARD, Target = ") +
                    String(targetWaistPosition) + String(" Current = ") + String(currentWaistPosition) +
                    String(" Speed = ") + String(waistMoveSpeed);
      debugString.toCharArray(debugStringChar, DEBUG_STRING_LEN );
      nh.loginfo(debugStringChar);

      if (waistMoveSpeed > WAIST_MOTOR_SPEED_MAX) {
        waistMoveSpeed = WAIST_MOTOR_SPEED_MAX; // Limit max speed
      }
      else if (waistMoveSpeed < WAIST_MOTOR_SPEED_SLOW) {
        waistMoveSpeed = WAIST_MOTOR_SPEED_SLOW;  // Assure minimun speed
      }
      if ( !waistLimitDown) {   // Don't exceed maximum throw of the actuator
        newWaistMotorCommand = waistMoveSpeed;//WAIST_MOTOR_SPEED_MED; // speed/direction ( -127 to 127)  <<<<<<< TODO - use calculated speed
      }
      debugString = String("DEBUG: Waist Move Speed = ") + String(waistMoveSpeed);
      debugString.toCharArray(debugStringChar, DEBUG_STRING_LEN );
      nh.loginfo(debugStringChar);

    }
    else if ( (waistPositionDelta < 0) && (waistPositionDelta < -WAIST_POSITION_HYSTERESIS) ) {
      // Move up/back
      nh.loginfo("DEBUG: Waist MOVE BACK!");

      if (waistMoveSpeed < -WAIST_MOTOR_SPEED_MAX) {
        waistMoveSpeed = -WAIST_MOTOR_SPEED_MAX; // Limit max speed
      }
      if (waistMoveSpeed > -WAIST_MOTOR_SPEED_SLOW) {
        waistMoveSpeed = -WAIST_MOTOR_SPEED_SLOW; // Assure minimun speed
      }
      if ( !waistLimitUp) {   // Don't exceed maximum throw of the actuator
        newWaistMotorCommand = waistMoveSpeed; // -WAIST_MOTOR_SPEED_MED; // speed/direction ( -127 to 127)  <<<<<<< TODO - use calculated speed
      }
      debugString = String("DEBUG: Waist Move Speed = ") + String(waistMoveSpeed);
      debugString.toCharArray(debugStringChar, DEBUG_STRING_LEN );
      nh.loginfo(debugStringChar);

    }
    else {
      // At target.  Stop the motors if moving
      waistRosMoveInProgress = false; // end the move, so others can take control, such as bluetooth phone
      newWaistMotorCommand = WAIST_MOTOR_SPEED_STOP;
    }


  }

  //////////////////////////////////////////////////////////////
  // Bluetooth command from Phone
  meetAndroid.receive(); // check for Bluetooth events
  if (Android.Connected)
  {
    //nh.loginfo("Android Connected");

    if (Android.Cmd != 0) // lastAndroidCmd) 
    {
      // New command received from phone
      nh.loginfo("ARDUINO: Android Command Received.  Key = ");
      debugString = String(Android.Cmd);
      debugString.toCharArray(debugStringChar, 10 );
      nh.loginfo(debugStringChar);
      if ((9 == Android.Cmd) && BT_WAIST_DEBUG) {
        // Manual control, Waist Down
        newWaistMotorCommand = WAIST_MOTOR_SPEED_MED;
        debugString = String("ARDUINO SENDING WAIST MOTOR CMD: ") + String(newWaistMotorCommand);
        debugString.toCharArray(debugStringChar, DEBUG_STRING_LEN );
        nh.loginfo(debugStringChar);
        if (waistRosMoveInProgress) {
          nh.loginfo("ARDUINO: Android Command may not work. waistRosMoveInProgress");
        }
      }
      else if ((10 == Android.Cmd)  && BT_WAIST_DEBUG) {
        // Manual control, Waist Up
        newWaistMotorCommand = -WAIST_MOTOR_SPEED_MED;
        debugString = String("ARDUINO SENDING WAIST MOTOR CMD: ") + String(newWaistMotorCommand);
        debugString.toCharArray(debugStringChar, DEBUG_STRING_LEN );
        nh.loginfo(debugStringChar);
        if (waistRosMoveInProgress) {
          nh.loginfo("ARDUINO: Android Command may not work. waistRosMoveInProgress");
        }
      }
      else if ((11 == Android.Cmd) && BT_WAIST_DEBUG) {
        // Manual control, Waist Stop
        newWaistMotorCommand = WAIST_MOTOR_SPEED_STOP;
        debugString = String("ARDUINO SENDING WAIST MOTOR CMD: ") + String(newWaistMotorCommand);
        debugString.toCharArray(debugStringChar, DEBUG_STRING_LEN );
        nh.loginfo(debugStringChar);
        if (waistRosMoveInProgress) {
          nh.loginfo("ARDUINO: Android Command may not work. waistRosMoveInProgress");
        }
      }
      else if(0 != Android.Cmd) { // ignore connect
        
        // Send bluetooth command to behavior controler
        switch (Android.Cmd) {
          case 1:
            behavior_cmd_msg.commandState = "WAVE";
            break;
          case 2:
            behavior_cmd_msg.commandState = "HEAD_CENTER";
            break;
          case 3:
            behavior_cmd_msg.commandState = "WAKEUP";
            break;
          case 4:
            behavior_cmd_msg.commandState = "SLEEP";
            break;
            
          case 5:
            behavior_cmd_msg.commandState = "FOLLOW_ME";
            break;
          case 6:
            behavior_cmd_msg.commandState = "TELL_JOKE";
            behavior_cmd_msg.param1 = "STAR WARS"; 
            break;
          case 7:
            behavior_cmd_msg.commandState = "HANDS_UP";
            break;
          case 8:
            behavior_cmd_msg.commandState = "SHAKE_HANDS";
            break;

          case 9:
            behavior_cmd_msg.commandState = "BOW"; // "SHIP"; // "TEST_JOINTS"; // "HAVE_SOMETHING";
            break;
          case 10:
            behavior_cmd_msg.commandState = "SHIP";
            break;
          case 11:
            behavior_cmd_msg.commandState = "RUN_SCRIPT"; 
            behavior_cmd_msg.param1 = "believer_short"; // run "believer.csv"
            break;
          case 12:
            behavior_cmd_msg.commandState = "INTRO";
            break;
          case 13:
            // FORCE STOP
            bt_motor_cmd_msg.axes[0] = 0.0;
            bt_motor_cmd_msg.axes[1] = 0.0;
            bt_motor_cmd_msg.header.stamp = nh.now();
            pub_bluetooth_motor_cmd.publish(&bt_motor_cmd_msg);
            behavior_cmd_msg.commandState = "STOP";
            break;
          case 14:
            behavior_cmd_msg.commandState = "TELL_TIME";
            break;
          case 15:
            behavior_cmd_msg.commandState = "DANGER";
            break;

          default:
            debugString = String("UNHANDLED ARDUINO COMMAND BUTTON NUMBER: ") + String(Android.Cmd);
            debugString.toCharArray(debugStringChar, DEBUG_STRING_LEN );
            nh.logwarn(debugStringChar);
            behavior_cmd_msg.commandState = "UNHANDLED";
        }
        
        debugString = String("ARDUINO: Sending Command: ") + String(behavior_cmd_msg.commandState);
        debugString.toCharArray(debugStringChar, DEBUG_STRING_LEN );
        nh.loginfo(debugStringChar);
         
        pub_behavior_cmd.publish( &behavior_cmd_msg );

      } // if (Android.Cmd != lastAndroidCmd)

      Android.Cmd = 0;
      //lastAndroidCmd = Android.Cmd;
    }

    if (Android.AccEnabled ) {     // && Android.UpdatePending)
      //nh.loginfo("ARDUINO: Android ACC Update");
      // Accelerometer enabled, publish latest values as Joystick messages
      // values should be in the range of -1.0 to +1.0
      bt_motor_cmd_msg.buttons[4] = 1;  // Enable Deadman Switch      
      bt_motor_cmd_msg.axes[1] = Android.Pitch;;
      bt_motor_cmd_msg.axes[0] = Android.Roll;
      bt_motor_cmd_msg.header.stamp = nh.now();
      pub_bluetooth_motor_cmd.publish(&bt_motor_cmd_msg);
      //debugString = String("ARDUINO: BlueTooth Pitch: ") + String(Android.Pitch) + String(" Roll: ") + String(Android.Roll);
      //debugString.toCharArray(debugStringChar, DEBUG_STRING_LEN );
      //nh.loginfo(debugStringChar);

    }
    Android.UpdatePending = false; // reset flag
  }


  //////////////////////////////////////////////////////////////
  // Send waist motor command if changed
  //nh.loginfo("DEBUG: Checking waist motor command");
  sendWaistMotorCommand(newWaistMotorCommand);




  //////////////////////////////////////////////////////////////
  // Publish Status - publish everything every so often... ROS does not like old data, so just republish
  // could potentially use "latch=true"

  
  // Check to see if it's time to publish status
  if ( TimeToPublishStatus() ) {
    digitalWrite(STATUS_LED_PIN, LOW);   // Active Low
    //SetArmLED( I2C_PCF8574_LEFT_ARM, HIGH ); // blink the arms too
    //SetArmLED( I2C_PCF8574_RIGHT_ARM, HIGH );
    //ReadSensors();
    //ResetSensors();

    // Publish waist position data.  If not yet initalized, this will be 0.0
    // we publish even if not initialized, so robot model links will initialize
    //pub_knee_position.publish(&kneePositionMsg);
    //pub_hip_position.publish(&hipPositionMsg);

    waist_joint_states_msg.header.stamp = nh.now();
    pub_waist_joint_states.publish(&waist_joint_states_msg);
  
    // Should publish rate be limited to TimeToPublishStatus ?
    // Publish Compass value
    unsigned int compassReading = Read_Compass();
    compassMsg.data = (float)compassReading / 10.0; // compass reported in tenth degrees
    pub_compass.publish(&compassMsg);

    // Publish IMU orientation value
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    imuOrientationMsg.x = euler.x();
    imuOrientationMsg.y = euler.y();
    imuOrientationMsg.z = euler.z();

    if( (0.0 != imuOrientationMsg.x) || (0.0 != imuOrientationMsg.y) || (0.0 != imuOrientationMsg.z) ) {
      // waits for the IMU to be ready before publishing
      pub_imu_orientation.publish(&imuOrientationMsg);
    }

    // Publish ambient temperature as provided by the IMU
    imuTemperatureMsg.data = bno.getTemp();
    if( 0.0 != imuTemperatureMsg.data ) {
      pub_imu_temperature.publish(&imuTemperatureMsg);
    }
  }

  //////////////////////////////////////////////////////////////
  // Blink Heartbeat LED if it's time
  if ( TimeToBlinkHeartBeat() ) {
    digitalWrite(HEARTBEAT_LED_PIN, HIGH); // Active High
    digitalWrite(11, HIGH); // Active High
  }

  // ROS Heartbeat / Communicate with ROS
  nh.spinOnce();
  /***
    // keep loop to a tight 20ms
    int LoopTime = millis() - LoopStartTime;

    if ( LoopTime > LOOP_TIME_MS )
    {
      // Enable this to test/assure that the loop completes in less than 20ms
      // Disabled for now becuase it reports a lot of 21/22 - need to fix this
      // PrintDebugDec("LoopTime = ", LoopTime);
    }
    else
    {
      while ( LoopTime < (LOOP_TIME_MS - 5) )
      {
        // read sensors instead of sitting idle
        // Read_Next_Analog_Port();    // read an analog port (need time between reads)
        // read the head sensor, so we don't miss short touches
        // Status.HeadSensor |= ReadPCF8574(I2C_PCF8574_HEAD); // read capacitive sensor in head (just a touch sensor)
        delay(1);
        LoopTime = millis() - LoopStartTime; // see how much time is still left
      }
      delay( LOOP_TIME_MS - LoopTime );
    }
  ***/


} // End of Looop
