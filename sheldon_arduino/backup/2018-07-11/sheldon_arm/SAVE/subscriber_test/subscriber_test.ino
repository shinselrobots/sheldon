/*
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */
#define USE_USBCON  // NEEDED FOR ATmega32u4 - FEATHER OR LEONARDO!!

#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
char hello[13] = "hello world!";

void messageCb(const std_msgs::UInt16& cmd_msg){

  if(0 == cmd_msg.data) {
    nh.loginfo("Arduino: LED COMMAND = OFF");
    digitalWrite(13, LOW);   // blink the led
  }
  else {
    nh.loginfo("Arduino: LED COMMAND = ON");
    digitalWrite(13, HIGH);   // blink the led
 
  }
  
}

//ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb );
ros::Subscriber<std_msgs::UInt16> sub("toggle_led", &messageCb);

void setup()
{
  pinMode(13, OUTPUT);
  // Serial.begin(57600);
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);

  digitalWrite(13, HIGH);   // led on


}

void loop()
{
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(10);
}
