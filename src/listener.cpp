#include <iostream>
using namespace std;

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <kobuki_msgs/BumperEvent.h>

void chatterCallback(const kobuki_msgs::BumperEventConstPtr msg)
{
  if(msg->state == kobuki_msgs::BumperEvent::PRESSED){
 	ROS_INFO("PRESSED");
  }
  else{
	ROS_INFO("RELEASED");
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "itb_listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/mobile_base/events/bumper", 1000, chatterCallback);

  ros::spin();
  return 0;
}
