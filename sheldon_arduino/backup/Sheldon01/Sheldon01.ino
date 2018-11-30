#include <Wire.h> // for I2C
#include <Encoder.h>  // for reading Quadrature Encoders
#include <MeetAndroid.h>
//#include <Adafruit_ADS1015.h>
//#include <Kangaroo.h>
#include <Sabertooth.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <ros/time.h>
#include "RobotConstants.h"

// Constants
#define PI                          3.1415926535
#define WAIST_TICKS_MAX             106.0  // number of ticks leaning forward from calibrated "zero" point (standing straight up)
#define WAIST_DEGREES_MAX           58.0

// DEBUG ONLY: Enable for debug to control waist from BT Phone, keys 9,10,11.
const boolean BT_WAIST_DEBUG = 0;  

const float WAIST_TICKS_TO_RADIANS    = (PI * WAIST_DEGREES_MAX) / (180 * WAIST_TICKS_MAX); // calculate at comple time for efficiency
const float WAIST_RADIANS_TO_TICKS    = (180 * WAIST_TICKS_MAX) / (PI * WAIST_DEGREES_MAX);
const int   WAIST_TICKS_HOME          = 3; // number of ticks from calibrated "zero" point to home (standing straight up)

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
//Adafruit_ADS1015  ads1015;

//int               waistCalibrateState = WAIST_NOT_CALIBRATED;
WAIST_CALIBRATE_STATE_T waistCalibrateState = WAIST_NOT_CALIBRATED;

int               targetWaistPosition = 0;  // Ticks from *cal* position
int               currentWaistPosition = 0; // Ticks from *cal* position
int               newWaistMotorCommand = 0;
int               lastWaistMotorCommand = 0;
int               waistRampSpeed = 0;
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
ros::NodeHandle   nh;


// ROS node subscribers and publishers
void waist_calibrate_cmd_callback(const std_msgs::Empty& cmd_msg) {
  nh.loginfo("Arduino Got WAIST COMMAND CALIBRATE message");
  waistCalibrateState = WAIST_CAL_BEGIN;
}
ros::Subscriber<std_msgs::Empty> wasteCalibrateSubscriber("\waist_calibrate", &waist_calibrate_cmd_callback); // does this need & ?

void waist_position_cmd_callback(const std_msgs::Float32& cmd_msg) {
  // waist position is in radians from *Home*.  must convert to Ticks from *cal*.
  if (WAIST_CAL_COMPLETED == waistCalibrateState) {
    nh.loginfo("Arduino Got WAIST COMMAND message");
    targetWaistPosition = (int)(((float)cmd_msg.data * WAIST_RADIANS_TO_TICKS) + 0.5) + WAIST_TICKS_HOME; // 0.5 to avoid roundoff error
    waistRampSpeed = 0;
    waistRosMoveInProgress = true;  // allow ramp controller to take control
    debugString = String("DEBUG: Waist Target Ticks = ") + String(targetWaistPosition);
    debugString.toCharArray(debugStringChar, DEBUG_STRING_LEN );
    nh.loginfo(debugStringChar);
  }
  else {
    nh.loginfo("Arduino WAIST COMMAND message IGNORED!  Not Calibrated!");
  }
}
ros::Subscriber<std_msgs::Float32> waistPositionSubscriber("\waist_goal_position", &waist_position_cmd_callback); // does this need & ?


// Sabertooth motor control for Waists and Hip
Sabertooth ST(128, Serial2); // Address 128 (default), but use Serial2 as the serial port.

// Assign pins for quadrature encoder.
// Mega pins with interrupt are: 2,3, plus 18,19,20,21 (reserved for Serial1 and I2C)
Encoder waistEncoder(KNEE_ENCODER_A_PIN, KNEE_ENCODER_B_PIN);


std_msgs::Float32 waistPositionMsg;
ros::Publisher pub_waist_position("/waist_current_position", &waistPositionMsg);

sensor_msgs::Joy bt_motor_cmd_msg;
ros::Publisher pub_bluetooth_motor_cmd("/joy", &bt_motor_cmd_msg);

void sendWaistMotorCommand(int newCommand)
{
  if ( newCommand != lastWaistMotorCommand ) {
    debugString = String("DEBUG: Sending waist motor command: ") + String(newCommand);
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
  nh.loginfo("ROS Init Start");
  nh.subscribe(wasteCalibrateSubscriber);
  nh.subscribe(waistPositionSubscriber);
  nh.advertise(pub_waist_position);
  nh.advertise(pub_bluetooth_motor_cmd);
  // while(!nh.connected()) nh.spinOnce();
  //  nh.loginfo("Arduino ROS connected");

  // Serial.begin(19200);   // NOTE: USB Serial port communication with the PC handled by ROS node library
  Serial1.begin(115200);    // bluetooth connection with Android phone
  Serial2.begin(9600);      // Sabertooth motor control for Waists
  Wire.begin();             // For I2C

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
  bt_motor_cmd_msg.axes_length = NUMBER_OF_JOYSTICK_BUTTONS;
  bt_motor_cmd_msg.buttons[0] = 0;

  nh.loginfo("Arduino Init Complete");
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
      nh.loginfo("Waist Up Limit!");
      // Reached maximum throw of the actuator.
      //newWaistMotorCommand = WAIST_MOTOR_SPEED_STOP;
    }
    lastWaistLimitUp = waistLimitUp;
  }

  waistLimitDown = digitalRead(KNEE_LIMIT_DOWN_PIN);  // Active High
  if ( waistLimitDown != lastWaistLimitDown) {
    if (waistLimitDown) {
      nh.loginfo("Waist Down Limit!");
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
          nh.loginfo("WAIST_CAL_MOVE_DOWN_WAIT timer completed, going to next step");
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
          nh.loginfo("Waist Encoder Calibrated! <=========");
          // now, move to home position (standing straight up)
          targetWaistPosition = 0;
          newWaistMotorCommand = WAIST_MOTOR_SPEED_SLOW; // move forward/down
          waistCalibrateState = WAIST_CAL_MOVING_TO_HOME;
        }
        else {
          nh.loginfo("Waist Cal: MOVE UP Looking for Limit switch");
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
        debugString = String("ERROR! Bad Calibration State = ") + String(waistCalibrateState);
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
      nh.loginfo("******* WAIST CALIBRATION COMPLETED *******");
    }
  }

  if (WAIST_CAL_COMPLETED == waistCalibrateState) {
    // only report encoder values after it has been calibrated
    if (newEncoderValue != currentWaistPosition) {
      currentWaistPosition = newEncoderValue;
      // publish current position in Radians
      waistPositionMsg.data = (float)(currentWaistPosition - WAIST_TICKS_HOME) * WAIST_TICKS_TO_RADIANS;
      pub_waist_position.publish(&waistPositionMsg);
      debugString = String("DEBUG: Encoder Ticks = ") + String(newEncoderValue) + String("  Radians = ") + String(waistPositionMsg.data);
      debugString.toCharArray(debugStringChar, DEBUG_STRING_LEN );
      nh.loginfo(debugStringChar);
    }
  }


  //////////////////////////////////////////////////////////////
  // Handle Waist Move Command
  // move to target waist positon if we are not already there
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

    if (Android.Cmd != lastAndroidCmd) {
      // New command received from phone
      nh.loginfo("Android Command Received.  Key = ");
      debugString = String(Android.Cmd);
      debugString.toCharArray(debugStringChar, 10 );
      nh.loginfo(debugStringChar);
      if (13 == Android.Cmd) {
        // FORCE STOP
        bt_motor_cmd_msg.axes[0] = 0.0;
        bt_motor_cmd_msg.axes[1] = 0.0;
        bt_motor_cmd_msg.header.stamp = nh.now();
        pub_bluetooth_motor_cmd.publish(&bt_motor_cmd_msg);
      }
      else if ((9 == Android.Cmd) && BT_WAIST_DEBUG) {
        // Manual control, Waist Down
        newWaistMotorCommand = WAIST_MOTOR_SPEED_MED;
        debugString = String("ARDUINO SENDING WAIST MOTOR CMD: ") + String(newWaistMotorCommand);
        debugString.toCharArray(debugStringChar, DEBUG_STRING_LEN );
        nh.loginfo(debugStringChar);
        if (waistRosMoveInProgress) {
          nh.loginfo("Android Command may not work. waistRosMoveInProgress");
        }
      }
      else if ((10 == Android.Cmd)  && BT_WAIST_DEBUG){
        // Manual control, Waist Up
        newWaistMotorCommand = -WAIST_MOTOR_SPEED_MED;
        debugString = String("ARDUINO SENDING WAIST MOTOR CMD: ") + String(newWaistMotorCommand);
        debugString.toCharArray(debugStringChar, DEBUG_STRING_LEN );
        nh.loginfo(debugStringChar);
        if (waistRosMoveInProgress) {
          nh.loginfo("Android Command may not work. waistRosMoveInProgress");
        }
      }
      else if ((11 == Android.Cmd) && BT_WAIST_DEBUG) {
        // Manual control, Waist Stop
        newWaistMotorCommand = WAIST_MOTOR_SPEED_STOP;
        debugString = String("ARDUINO SENDING WAIST MOTOR CMD: ") + String(newWaistMotorCommand);
        debugString.toCharArray(debugStringChar, DEBUG_STRING_LEN );
        nh.loginfo(debugStringChar);
        if (waistRosMoveInProgress) {
          nh.loginfo("Android Command may not work. waistRosMoveInProgress");
        }
      }

      lastAndroidCmd = Android.Cmd;
    }

    if (Android.AccEnabled ) {     // && Android.UpdatePending)
      //nh.loginfo("Android ACC Update");
      // Accelerometer enabled, publish latest values as Joystick messages
      // values should be in the range of -1.0 to +1.0
      bt_motor_cmd_msg.axes[0] = Android.Pitch;;
      bt_motor_cmd_msg.axes[1] = Android.Roll;
      bt_motor_cmd_msg.header.stamp = nh.now();
      pub_bluetooth_motor_cmd.publish(&bt_motor_cmd_msg);
      debugString = String("DEBUG: BlueTooth Accelerometer Pitch: ") + String(Android.Pitch);
      debugString.toCharArray(debugStringChar, DEBUG_STRING_LEN );
      nh.loginfo(debugStringChar);

    }
    Android.UpdatePending = false; // reset flag
  }


  //////////////////////////////////////////////////////////////
  // Send waist motor command if changed
  //nh.loginfo("DEBUG: Checking waist motor command");
  sendWaistMotorCommand(newWaistMotorCommand);




  //////////////////////////////////////////////////////////////
  // Publish Other Stuff
  // Check to see if it's time to publish status
  if ( TimeToPublishStatus() ) {
    digitalWrite(STATUS_LED_PIN, LOW);   // Active Low
    //SetArmLED( I2C_PCF8574_LEFT_ARM, HIGH ); // blink the arms too
    //SetArmLED( I2C_PCF8574_RIGHT_ARM, HIGH );
    //ReadSensors();
    //ResetSensors();

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






