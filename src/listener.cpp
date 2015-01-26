#include <iostream>
using namespace std;

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/CliffEvent.h>

void chatterCallback(const kobuki_msgs::BumperEventConstPtr msg)
{
  if(msg->state == kobuki_msgs::BumperEvent::PRESSED){
 	ROS_INFO("PRESSED");
  }
  else{
	ROS_INFO("RELEASED");
  }

  if(msg->state == kobuki_msgs::CliffEventConstPtr msg){
	ROS_INFO("CLIFF");
  }
  else{
	ROS_INFO("FLOOR");
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "itb_listener");

  ros::NodeHandle n;

  ros::Subscriber sub_bumper = n.subscribe("/mobile_base/events/bumper", 1000, chatterCallback);

  ros::Subscriber sub_cliff = n.subscriber("mobile_base/events/cliff", 1000, chatterCallback);

  ros::spin();

  return 0;
}
