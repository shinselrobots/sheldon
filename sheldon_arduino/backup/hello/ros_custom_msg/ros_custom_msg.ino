/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

//#define USE_USBCON // NEEDED FOR ATmega32u4 - FEATHER OR LEONARDO!!
//#define USBCON // NEEDED FOR ATmega32u4 - FEATHER OR LEONARDO!!
#include <ros.h>
#include <std_msgs/String.h>
#include <wheel_control/WheelOdomRaw.h>

ros::NodeHandle nh;
wheel_control::WheelOdomRaw odom_msg;
ros::Publisher odom_pub("wheel_odom_raw", &odom_msg);

long right_wheel_count = 0L;
long left_wheel_count = 0L;
int right_wheel_speed = 0;
int left_wheel_speed = 0;

void setup()
{
  //nh.getHardware()->setBaud(19200);
  nh.initNode();
  nh.advertise(odom_pub);
  pinMode(LED_BUILTIN, OUTPUT);
  
}

void loop()
{
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  
  odom_msg.odom_ticks_right += 10;
  odom_msg.odom_ticks_left -= 10;
  odom_msg.speed_ticks_right = 50;
  odom_msg.speed_ticks_left = -50;
  
  odom_pub.publish( &odom_msg );
  
  nh.spinOnce();
  //delay(200);                       
  //nh.spinOnce();
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(200);   
  //nh.spinOnce();
}
