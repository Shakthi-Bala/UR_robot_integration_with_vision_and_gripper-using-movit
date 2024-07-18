/*
 * rosserial PubSub Example
 * Prints "hello world!" and toggles led
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

ros::NodeHandle  nh;

// setpoint contains incoming message
int setpoint = 0;

void messageCb( const std_msgs::Bool& msg){
  setpoint = (int)msg.data;
}

ros::Subscriber<std_msgs::Bool> sub("gripper", messageCb );



std_msgs::Bool received_msg;
ros::Publisher chatter("gripper_state", &received_msg);

void setup()
{
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
}

void loop()
{
  received_msg.data = setpoint;

  // change pin outputs using setpoint here.
  
  chatter.publish( &received_msg );
  nh.spinOnce();
  delay(50);
}
