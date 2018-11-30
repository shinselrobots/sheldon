/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */
 
#define USE_USBCON // NEEDED FOR ATmega32u4 - FEATHER OR LEONARDO!!

#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
long odom_ticks_right = 2000000000L;
long odom_ticks_left = 0L;
int speed_ticks_right = 50;
int speed_ticks_left = -50;

void setup()
{
  //nh.getHardware()->setBaud(9600);
  nh.initNode();
  nh.advertise(chatter);
}

void loop()
{
  odom_ticks_right += 10;
  odom_ticks_left -= 10;
  String msgString = String(odom_ticks_right) + " " + String(odom_ticks_left) + " "  + String(speed_ticks_right) + " " + String(speed_ticks_left);
  
  str_msg.data = msgString.c_str(); // odom_char; // send data as a string of characters
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(100);
}
