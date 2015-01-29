#include <iostream>

using namespace std;

#include "ros/ros.h"
#include <std_msgs/String.h>
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/ButtonEvent.h>
#include <kobuki_msgs/CliffEvent.h>
#include <kobuki_msgs/Led.h>
#include <kobuki_msgs/WheelDropEvent.h>
#include <kobuki_msgs/MotorPower.h>
#include <geometry_msgs/Twist.h>

#include <unistd.h>

ros::NodeHandle* node;
ros::Subscriber 
	sub_bumper,
	sub_button,
	sub_cliff,
	sub_led1,
	sub_led2,
	sub_wheel;
ros::Publisher
	pub_motor,
	pub_velocity,
	pub_led1,
	pub_led2,
	pub_shellcmd;



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
			
			std_msgs::String msg;
			msg.data = "play ~/hey.ogg &";
			pub_shellcmd.publish(msg);
		}
	}
}

void cliffCallback(const kobuki_msgs::CliffEventConstPtr msg){
	if(msg->state == kobuki_msgs::CliffEvent::CLIFF){
		std_msgs::String msg;
		msg.data = "play ~/nonono.ogg &";
		pub_shellcmd.publish(msg);
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

void wheelCallback(const kobuki_msgs::WheelDropEventConstPtr msg){
	if(msg->wheel == kobuki_msgs::WheelDropEvent::LEFT){
		if(msg->state == kobuki_msgs::WheelDropEvent::RAISED){
			ROS_INFO("LEFT WHEEL RAISED");
		}
		else if(msg->state == kobuki_msgs::WheelDropEvent::DROPPED){
			ROS_INFO("LEFT WHEEL DROPPED");
		}
	}
	else if(msg->wheel == kobuki_msgs::WheelDropEvent::RIGHT){
		if(msg->state == kobuki_msgs::WheelDropEvent::RAISED){
			ROS_INFO("RIGHT WHEEL RAISED");
		}
		else if(msg->state == kobuki_msgs::WheelDropEvent::DROPPED){
			ROS_INFO("RIGHT WHEEL DROPPED");
		}
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
	sub_wheel = node->subscribe("/mobile_base/events/wheel_drop", 1000, wheelCallback);


	pub_velocity = node->advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
	pub_led1 = node->advertise<kobuki_msgs::Led>("/mobile_base/commands/led1", 1000);
	pub_led2 = node->advertise<kobuki_msgs::Led>("/mobile_base/commands/led2", 1000);
	pub_shellcmd = node->advertise<std_msgs::String>("itb_shellcmd_topic", 1000);

	ros::spin();

	return 0;
}

