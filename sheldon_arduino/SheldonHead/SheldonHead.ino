// Sheldon Head controls Eyes currently, and maybe sensors in the future

// Adafruit NeoPixel used for the eyes
// IMPORTANT: To reduce NeoPixel burnout risk, add 1000 uF capacitor across
// pixel power leads, add 300 - 500 Ohm resistor on first pixel's data input
// and minimize distance between Arduino and first pixel.  Avoid connecting
// on a live circuit...if you must, connect GND first.

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

// ROS
#include <ros.h>
//#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>
//#include <std_msgs/Empty.h>
//#include <geometry_msgs/Twist.h>
//#include <sensor_msgs/Joy.h>
//#include <behavior_common/CommandState.h>
#include <ros/time.h>
//#include "RobotConstants.h"


/////////////////////////////////////////////////////////////////////////////////////
// Constants

#define NeoPIN 9 // default is pin 10
#define PIXELS_PER_RING 24
#define DEFAULT_EYE_BRIGHTNESS 127 // med brightness
const uint32_t BLACK = 0;

// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)

enum EYE_CMD_STATE_T {
  EYES_OFF = 0,
  EYES_ON_SOLID,
  EYES_AUTO_BLINK,
};


/////////////////////////////////////////////////////////////////////////////////////
// Global Variables

EYE_CMD_STATE_T EyeCmdState = EYES_OFF;

Adafruit_NeoPixel strip = Adafruit_NeoPixel((PIXELS_PER_RING * 2), NeoPIN, NEO_GRB + NEO_KHZ800);

typedef struct 
 {
     uint8_t r;
     uint8_t g;
     uint8_t b;
 } eyeColor_t;

eyeColor_t eyeColor = {0, 0, 0};
bool blinkNow = false; // force wait loop to exit immediately
ros::NodeHandle   nh;


// ROS node subscribers and publishers

// EYE MODE: To test, try this: rostopic pub -1 /head/eye_cmd std_msgs/UInt16  0 // Eyes Off
void eye_cmd_callback(const std_msgs::UInt16& cmd_msg) {
  if(0 == cmd_msg.data) {
  nh.loginfo("Head Arduino: EYE COMMAND = OFF");
    EyeCmdState = EYES_OFF;
    EyesOff();
  }
  else if(1 == cmd_msg.data) {
  nh.loginfo("Head Arduino: EYE COMMAND = ON");
    EyeCmdState = EYES_ON_SOLID;
    EyesOn(strip.Color(eyeColor.r, eyeColor.g, eyeColor.b));
  }
  else {
    nh.loginfo("Head Arduino: EYE COMMAND = BLINK");
    EyeCmdState = EYES_AUTO_BLINK;
  }
}
ros::Subscriber<std_msgs::UInt16> eyeCommandSubscriber("/head/eye_cmd", &eye_cmd_callback);

// EYE COLOR: To test, try this: rostopic pub -1 /head/eye_color std_msgs/UInt32 0x2f2f2f (dim white)
void eye_color_callback(const std_msgs::UInt32& cmd_msg) {
  uint32_t color = cmd_msg.data;

  if( 0x00002f == color) {
    nh.loginfo("Head Arduino Got EYE COLOR message: DEFAULT COLOR");
  }
  else {
    nh.loginfo("Head Arduino Got EYE COLOR message: CUSTOM COLOR");
  }
  eyeColor.b = color & 0xFF;
  eyeColor.g = (color >> 8) & 0xFF;
  eyeColor.r = (color >> 16) & 0xFF;
  blinkNow = true; // force wait loop to exit immediately

}

ros::Subscriber<std_msgs::UInt32> eyeColorSubscriber("/head/eye_color", &eye_color_callback);


////////////////////////////////////////////////////////////////////////////////
void setup() {
  // This is for Trinket 5V 16MHz, you can remove these three lines if you are not using a Trinket
#if defined (__AVR_ATtiny85__)
  if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
#endif
  // End of trinket special code

  pinMode(LED_BUILTIN, OUTPUT); // heartbeat LED
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  randomSeed(analogRead(0));

  eyeColor = {0, 0, DEFAULT_EYE_BRIGHTNESS}; // default to blue
  // eyeColor = {0,  DEFAULT_EYE_BRIGHTNESS, 0}; // default to green
  // eyeColor = {0,  DEFAULT_EYE_BRIGHTNESS, DEFAULT_EYE_BRIGHTNESS}; // aqua
  // eyeColor = {DEFAULT_EYE_BRIGHTNESS, DEFAULT_EYE_BRIGHTNESS, DEFAULT_EYE_BRIGHTNESS}; // default to white

  // Initialize ROS
  nh.initNode();
  nh.loginfo("ROS Init Start");
  nh.subscribe(eyeCommandSubscriber);
  nh.subscribe(eyeColorSubscriber);

  // indicate startup
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on
    delay(300);
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED off
    delay(300);
  }
  
  EyeCmdState = EYES_OFF; // Off by default, until ROS turns them on
  //EyesOn(strip.Color(eyeColor.r, eyeColor.g, eyeColor.b) );
  
  nh.loginfo("Head Arduino started");
}


////////////////////////////////////////////////////////////////////////////////
void loop() {


  if(EYES_AUTO_BLINK == EyeCmdState) {
    EyesBlink(strip.Color(eyeColor.r, eyeColor.g, eyeColor.b), 5); // Blue, delay
  }
  
  // Random delays between blinks, between 1 and 6 seconds
  int randTime = random(1, 60);

  for(int i=0; i<randTime; i++) {
    // ROS Heartbeat / Communicate with ROS
    nh.spinOnce();
    delay(100);
    if(blinkNow) {
      blinkNow = false; // reset flag
      break; // exit wait loop
    }
  }
  
}


////////////////////////////////////////////////////////////////////////////////
// Subroutines

// EYES ON
// Quickly fill all the dots with a color.  Is there a better function for this?
void EyesOn(uint32_t c) {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
  }
  strip.show(); // move to end of loop?
}


// EYES OFF
// Quickly fill all the dots with a color.  Is there a better function for this?
void EyesOff() {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(0, 0, 0));
  }
  strip.show(); // move to end of loop?
}

// WAKE UP
void WakeUp(uint8_t wait) {

  for (uint16_t i = 0; i < 255; i++) {
    uint16_t LeftEyeRightSide = i;
    uint16_t LeftEyeLeftSide = (PIXELS_PER_RING - 1) - i;
    uint16_t RightEyeRightSide = PIXELS_PER_RING + i;
    uint16_t RightEyeLeftSide = ((PIXELS_PER_RING * 2) - 1) - i;

    strip.setPixelColor(LeftEyeRightSide, BLACK);
    strip.setPixelColor(LeftEyeLeftSide, BLACK );
    strip.setPixelColor(RightEyeRightSide, BLACK);
    strip.setPixelColor(RightEyeLeftSide, BLACK );
    strip.show();
    delay(wait);
  }
}

////////////////////////////////////////////////////////////
// BLINK EYES!
// Assumes eyes are on.  Blink Off then back On
// 24 LEDs per ring, 12 each side, 2 rings
const int LeftRingStart = 0;
const int RightRingStart = PIXELS_PER_RING;
const int HALF_RING = PIXELS_PER_RING / 2;

void EyesBlink(uint32_t c, uint8_t wait) {

  // Turn LEDs Off
  // for(int16_t i=(HALF_RING-1); i>=0; i--) {
  for (uint16_t i = 0; i < HALF_RING; i++) {
    uint16_t LeftEyeRightSide = i;
    uint16_t LeftEyeLeftSide = (PIXELS_PER_RING - 1) - i;
    uint16_t RightEyeRightSide = PIXELS_PER_RING + i;
    uint16_t RightEyeLeftSide = ((PIXELS_PER_RING * 2) - 1) - i;

    strip.setPixelColor(LeftEyeRightSide, BLACK);
    strip.setPixelColor(LeftEyeLeftSide, BLACK );
    strip.setPixelColor(RightEyeRightSide, BLACK);
    strip.setPixelColor(RightEyeLeftSide, BLACK );
    strip.show();
    delay(wait);
  }
  delay(100);

  // Turn LEDs Back ON
  for (int16_t i = (HALF_RING - 1); i >= 0; i--) {
    //  for(uint16_t i=0; i<HALF_RING; i++) {
    uint16_t LeftEyeRightSide = i;
    uint16_t LeftEyeLeftSide = (PIXELS_PER_RING - 1) - i;
    uint16_t RightEyeRightSide = PIXELS_PER_RING + i;
    uint16_t RightEyeLeftSide = ((PIXELS_PER_RING * 2) - 1) - i;

    strip.setPixelColor(LeftEyeRightSide, c);
    strip.setPixelColor(LeftEyeLeftSide, c );
    strip.setPixelColor(RightEyeRightSide, c);
    strip.setPixelColor(RightEyeLeftSide, c );
    strip.show();
    delay(wait);
  }
}

