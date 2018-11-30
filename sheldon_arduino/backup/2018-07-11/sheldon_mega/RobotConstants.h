/////////////////////////////////////////////////////////////////////////////////////
//  Robot Constants
/////////////////////////////////////////////////////////////////////////////////////

/*  General and Hardware Comments
 *  When using long wires with I2C, strong pull-up resistors required! 1k - 4.7k
 * I2C-IT sensors are available from Solarbotics
 */
 
#define LOOP_TIME_MS      20  // main loop should always take about this long (milliseconds)
#define NUM_RANGE_SENSORS  4
//#define BINARY(a,b,c,d,e,f,g,h)  (a<<7|b<<6|c<<5|d<<4|e<<3|f<<2|g<<1|h)
#define NOT_SET -99999  // unset values

// Status Structure
typedef struct
  {
    byte PortA;      
    byte PortC;    
    byte HeadSensor;
    int Compass;
    int OdometerLeft;   
    int OdometerRight;
    byte LeftHandPressureL;
    byte LeftHandPressureR;
    byte RangeSensor[NUM_RANGE_SENSORS];
  } ARDUINO_STATUS_T;

typedef struct
  {
    boolean Connected;
    boolean AccEnabled;
    boolean UpdatePending;      
    byte Cmd;
    float Compass;  
    float Roll;
    float Pitch;
  } ANDROID_STATUS_T;


////////////////////////////////////////////////////////////////
// Hardware Pins
//const int    INT_0 = 0;  // Interrupt pins for the Mega 2560, used for Encoder 
//const int    INT_1 = 1;
const int    KNEE_ENCODER_A_PIN =      3;   // EncoderA
const int    KNEE_ENCODER_B_PIN =      2;   // EncoderB
const int    KNEE_LIMIT_UP_PIN =       8;   // Limit switch at top of knee travel
const int    KNEE_LIMIT_DOWN_PIN =     9;   // Limit switch at bottom of knee travel

const int    INPUT_PIN_4 =             4;   // Wired with pullups, but not used
const int    INPUT_PIN_5 =             5;   // (Pins were for motor Odom, but did not work)

//const int    RELAY_BOARD_PIN_0 =   8;   // N/C - goes to relay control board
//const int    AUX_LIGHT_PIN =       9;   // Blue aux lights
//const int    SERVO_PWR_18V_PIN =  10;   // 18v Servo power for RX64
//const int    SERVO_PWR_12V_PIN =  11;   // 12v Servo power for all other servos

const int    HEARTBEAT_LED_PIN =  13;  // Yellow Heartbeat Led, active High
const int    STATUS_LED_PIN =     11;  // Red Status to PC indicator, active Low

const int    NUM_AD_IR_SENSORS =   6;  // Plugged into the Arduino Analog port AD0-14.  15 reserved for battery monitor.    
////////////////////////////////////////////////////////////////
// I2C Address Space Mapping

// I2C-IT IR Rangers (requires 2 consecutive addresses)
//const byte   I2C_IT_SENSOR_ADDR_0 =   0x10; // Left Hand IR Range
//const byte   I2C_IT_SENSOR_ADDR_1 =   0x12; // N/C
//const byte   I2C_IT_SENSOR_ADDR_2 =   0x14; // Right Hand IR Range
//const byte   I2C_IT_SENSOR_ADDR_3 =   0x16; // N/C
//const byte   I2C_IT_SENSOR_ADDR_4 =   0x20; // N/C
//const byte I2C_IT_SENSOR_ADDR_5 =   0x22; - In Use!
//const byte   I2C_IT_SENSOR_ADDR_6 =   0x24; // N/C
//const byte I2C_IT_SENSOR_ADDR_7 =   0x26; - In Use (blocked)

//const byte   I2C_PCF8574_LEFT_ARM =   0x21; // Left Arm Digital I/O
//const byte   I2C_PCF8574_HEAD =       0x22; // Head Digital I/O
//const byte   I2C_PCF8574_RIGHT_ARM =  0x23; // Right Arm Digital I/O

#define   I2C_CMPS11_COMPASS_ADDR  0x60 // Devantech CMPS03 Compass
const uint8_t   COMPASS_ANGLE_8_REG =       1; // register for reading 8 bit compass value


////////////////////////////////////////////////////////////////
//	Reads HVW "I2C-IT"IR Range Sensor via I2C.
// Tell the I2C-It Sensor what Units to use for return values
const byte   I2C_IT_UNITS_INCHES =        1;
const byte   I2C_IT_UNITS_CENTEMETERS =   2;
const byte   I2C_IT_UNITS_RAW =           3;


////////////////////////////////////////////////////////////////
// Other constants
const long   BlinkInterval =   600; // interval at which to blink the LED (milliseconds)
const long   PublishInterval =  100;// 333; // interval at which to send status to the PC (milliseconds)


/////////////////////////////////////////////////////////////////////////////////////
//  Robot Subroutines

boolean TimeToSendStatus();
boolean TimeToBlinkHeartBeat();
void ToggleHeartBeatLed();
void AllLedsOff();
void AllLedsOn();
void PrintDebug(char *Message);
//void PrintDebugHex(char *Message, int Value);
//void PrintDebugDec(char *Message, int Value);
void CounterISR0();
void CounterISR1();
//void ReadFastDigitalPorts();
//void ReadKneeEncoder( );
//byte Read_Digital_Port_A();
//byte Read_Digital_Port_C();
//void Read_I2C_IT_IR_Rangers();
//void Read_Hand_Pressure();
//void Read_Next_Analog_Port();
//boolean CheckPower(byte BatteryByte);
//void Read_Compass();
//byte I2C_Read_Byte( int deviceaddress, int datamode ); 
//void SetLedEyes( byte EyeState );
//void SetArmLED( byte DeviceAddress, boolean LedOn);
//byte ReadPCF8574(int deviceaddress);
void ReadSensors();
//void ResetSensors();




