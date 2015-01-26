#include <iostream>

using namespace std;

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/ButtonEvent.h>
#include <kobuki_msgs/CliffEvent.h>

void bumperCallback(const kobuki_msgs::BumperEventConstPtr msg){
	if(msg->state == kobuki_msgs::BumperEvent::LEFT){
		if(msg->state == kobuki_msgs::BumperEvent::RELEASED){
			ROS_INFO("BUMPER LEFT RELEASED");
		}
		else if(msg->state == kobuki_msgs::BumperEvent::PRESSED){
			ROS_INFO("BUMPER LEFT PRESSED");
		}
	}
	else if(msg->state == kobuki_msgs::BumperEvent::CENTER){
		if(msg->state == kobuki_msgs::BumperEvent::RELEASED){
			ROS_INFO("BUMPER CENTER RELEASED");
		}
		else if(msg->state == kobuki_msgs::BumperEvent::PRESSED){
			ROS_INFO("BUMPER CENTER PRESSED");
		}
	}
	else if(msg->state == kobuki_msgs::BumperEvent::RIGHT){
		if(msg->state == kobuki_msgs::BumperEvent::RELEASED){
			ROS_INFO("BUMPER RIGHT RELEASED");
		}
		else if(msg->state == kobuki_msgs::BumperEvent::PRESSED){
			ROS_INFO("BUMPER RIGHT PRESSED");
		}
	}
}

void buttonCallback(const kobuki_msgs::ButtonEventConstPtr msg){
  	if(msg->state == kobuki_msgs::ButtonEvent::Button0){
		if(msg->state == kobuki_msgs::ButtonEvent::RELEASED){
			ROS_INFO("BUTTON0 RELEASED");
		}
		else if(msg->state == kobuki_msgs::ButtonEvent::PRESSED){
			ROS_INFO("BUTTON0 PRESSED");
		}
	}
	else if(msg->state == kobuki_msgs::ButtonEvent::Button1){
		if(msg->state == kobuki_msgs::ButtonEvent::RELEASED){
			ROS_INFO("BUTTON1 RELEASED");
		}
		else if(msg->state == kobuki_msgs::ButtonEvent::PRESSED){
			ROS_INFO("BUTTON1 PRESSED");
		}
	}
	else if(msg->state == kobuki_msgs::ButtonEvent::Button2){
		if(msg->state == kobuki_msgs::ButtonEvent::RELEASED){
			ROS_INFO("BUTTON2 RELEASED");
		}
		else if(msg->state == kobuki_msgs::ButtonEvent::PRESSED){
			ROS_INFO("BUTTON2 PRESSED");
		}
	}
}

void cliffCallback(const kobuki_msgs::CliffEventConstPtr msg){
  	if(msg->state == kobuki_msgs::CliffEvent::LEFT){
		if(msg->state == kobuki_msgs::CliffEvent::FLOOR){
			ROS_INFO("SENSOR LEFT FLOOR");
		}
		else if(msg->state == kobuki_msgs::CliffEvent::CLIFF){
			ROS_INFO("SENSOR LEFT CLIFF");
		}
	}
	else if(msg->state == kobuki_msgs::CliffEvent::CENTER){
		if(msg->state == kobuki_msgs::CliffEvent::FLOOR){
			ROS_INFO("SENSOR CENTER FLOOR");
		}
		else if(msg->state == kobuki_msgs::CliffEvent::CLIFF){
			ROS_INFO("SENSOR CENTER CLIFF");
		}
	}
	else if(msg->state == kobuki_msgs::CliffEvent::RIGHT){
		if(msg->state == kobuki_msgs::CliffEvent::FLOOR){
			ROS_INFO("SENSOR RIGHT FLOOR");
		}
		else if(msg->state == kobuki_msgs::CliffEvent::CLIFF){
			ROS_INFO("SENSOR RIGHT CLIFF");
		}
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "itb_listener");

	ros::NodeHandle n;

	ros::Subscriber sub_bumper = n.subscribe("/mobile_base/events/bumper", 1000, bumperCallback);
	ros::Subscriber sub_button = n.subscribe("/mobile_base/events/button", 1000, buttonCallback);
	ros::Subscriber sub_cliff = n.subscribe("/mobile_base/events/cliff", 1000, cliffCallback);

	ros::spin();

	return 0;
}

