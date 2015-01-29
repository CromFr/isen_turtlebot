#include <iostream>

using namespace std;

#include "ros/ros.h"
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/ButtonEvent.h>
#include <kobuki_msgs/CliffEvent.h>
#include <kobuki_msgs/Led.h>
#include <kobuki_msgs/MotorPower.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle* node;
ros::Subscriber 
	sub_bumper,
	sub_button,
	sub_cliff,
	sub_led1,
	sub_led2;
ros::Publisher
	pub_motor,
	pub_velocity,
	pub_led1,
	pub_led2;



void bumperCallback(const kobuki_msgs::BumperEventConstPtr msg){
	if(msg->bumper == kobuki_msgs::BumperEvent::LEFT){
		if(msg->state == kobuki_msgs::BumperEvent::RELEASED){
			ROS_INFO("BUMPER LEFT RELEASED");
		}
		else if(msg->state == kobuki_msgs::BumperEvent::PRESSED){
			ROS_INFO("BUMPER LEFT PRESSED");
		}
	}
	else if(msg->bumper == kobuki_msgs::BumperEvent::CENTER){
		if(msg->state == kobuki_msgs::BumperEvent::RELEASED){
			ROS_INFO("BUMPER CENTER RELEASED");
		}
		else if(msg->state == kobuki_msgs::BumperEvent::PRESSED){
			ROS_INFO("BUMPER CENTER PRESSED");
		}
	}
	else if(msg->bumper == kobuki_msgs::BumperEvent::RIGHT){
		if(msg->state == kobuki_msgs::BumperEvent::RELEASED){
			ROS_INFO("BUMPER RIGHT RELEASED");
		}
		else if(msg->state == kobuki_msgs::BumperEvent::PRESSED){
			ROS_INFO("BUMPER RIGHT PRESSED");
		}
	}
}

void buttonCallback(const kobuki_msgs::ButtonEventConstPtr msg){
  	if(msg->button == kobuki_msgs::ButtonEvent::Button0){
		if(msg->state == kobuki_msgs::ButtonEvent::RELEASED){
			ROS_INFO("BUTTON0 RELEASED");
			kobuki_msgs::Led msg;
			msg.value = kobuki_msgs::Led::ORANGE;
			pub_led1.publish(msg);
		}
		else if(msg->state == kobuki_msgs::ButtonEvent::PRESSED){
			ROS_INFO("BUTTON0 PRESSED");
			kobuki_msgs::Led msg;
			msg.value = kobuki_msgs::Led::GREEN;
			pub_led1.publish(msg);
		}
	}
	else if(msg->button == kobuki_msgs::ButtonEvent::Button1){
		if(msg->state == kobuki_msgs::ButtonEvent::RELEASED){
			ROS_INFO("BUTTON1 RELEASED");
			geometry_msgs::Twist msg;
			msg.linear.x = 0.0;
			msg.angular.x = 0.0;
			pub_velocity.publish(msg);

		}
		else if(msg->state == kobuki_msgs::ButtonEvent::PRESSED){
			ROS_INFO("BUTTON1 PRESSED");
			geometry_msgs::Twist msg;
			msg.linear.x = 0.5;
			msg.angular.x = 0.0;
			pub_velocity.publish(msg);
		}
	}
	else if(msg->button == kobuki_msgs::ButtonEvent::Button2){
		if(msg->state == kobuki_msgs::ButtonEvent::RELEASED){
			ROS_INFO("BUTTON2 RELEASED");
		}
		else if(msg->state == kobuki_msgs::ButtonEvent::PRESSED){
			ROS_INFO("BUTTON2 PRESSED");
		}
	}
}

void cliffCallback(const kobuki_msgs::CliffEventConstPtr msg){
  	if(msg->sensor == kobuki_msgs::CliffEvent::LEFT){
		if(msg->state == kobuki_msgs::CliffEvent::FLOOR){
			ROS_INFO("SENSOR LEFT FLOOR");
		}
		else if(msg->state == kobuki_msgs::CliffEvent::CLIFF){
			ROS_INFO("SENSOR LEFT CLIFF");
		}
	}
	else if(msg->sensor == kobuki_msgs::CliffEvent::CENTER){
		if(msg->state == kobuki_msgs::CliffEvent::FLOOR){
			ROS_INFO("SENSOR CENTER FLOOR");
		}
		else if(msg->state == kobuki_msgs::CliffEvent::CLIFF){
			ROS_INFO("SENSOR CENTER CLIFF");
		}
	}
	else if(msg->sensor == kobuki_msgs::CliffEvent::RIGHT){
		if(msg->state == kobuki_msgs::CliffEvent::FLOOR){
			ROS_INFO("SENSOR RIGHT FLOOR");
		}
		else if(msg->state == kobuki_msgs::CliffEvent::CLIFF){
			ROS_INFO("SENSOR RIGHT CLIFF");
		}
	}
}

void ledCallback(const kobuki_msgs::LedConstPtr msg){
	if(msg->value == kobuki_msgs::Led::BLACK){
		ROS_INFO("BLACK LED");
	}
	else if(msg->value == kobuki_msgs::Led::GREEN){
		ROS_INFO("GREEN LED");
	}
	else if(msg->value == kobuki_msgs::Led::ORANGE){
		ROS_INFO("ORANGE LED");
	}
	else if(msg->value == kobuki_msgs::Led::RED){
		ROS_INFO("RED LED");
	}
}

int main(int argc, char **argv){
	ros::init(argc, argv, "itb_listener");
	node = new ros::NodeHandle;

	//Subscribe to sensors
	sub_bumper = node->subscribe("/mobile_base/events/bumper", 1000, bumperCallback);
	sub_button = node->subscribe("/mobile_base/events/button", 1000, buttonCallback);
	sub_cliff = node->subscribe("/mobile_base/events/cliff", 1000, cliffCallback);
	sub_led1 = node->subscribe("/mobile_base/commands/led1", 1000, ledCallback);
	sub_led2 = node->subscribe("/mobile_base/commands/led2", 1000, ledCallback);

	pub_velocity = node->advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
	pub_led1 = node->advertise<kobuki_msgs::Led>("/mobile_base/commands/led1", 1000);
	pub_led2 = node->advertise<kobuki_msgs::Led>("/mobile_base/commands/led2", 1000);

	ros::spin();

	return 0;
}

