/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

ros::NodeHandle  nh;

std_msgs::Bool msg;
ros::Publisher contact_pub("contact", &msg);
bool contact = false;

void setup()
{
  nh.initNode();
  nh.advertise(contact_pub);
  pinMode( 13, INPUT_PULLUP );
}

void loop()
{
  contact = digitalRead(13);
  msg.data = !contact;
  contact_pub.publish( &msg );
  nh.spinOnce();
  delay(1);
}
