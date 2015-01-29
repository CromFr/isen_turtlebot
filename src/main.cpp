#include <iostream>
#include <unistd.h>
using namespace std;

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/ButtonEvent.h>
#include <kobuki_msgs/CliffEvent.h>
#include <kobuki_msgs/Led.h>
#include <geometry_msgs/Twist.h>

class Controller;
Controller* ctrl;

ros::NodeHandle* node;
ros::Subscriber 
	sub_bumper,
	sub_button,
	sub_cliff;
ros::Publisher
	pub_velocity,
	pub_led1,
	pub_led2,
	pub_shellcmd;



class Controller
{
public:	
	Controller():
		cliff({false, false, false}),
		wall({false, false, false})
	{
		SetLed(1, Controller::color::BLACK);
		SetLed(2, Controller::color::BLACK);
		ctrl->Say("I, am, ready");
	}

	bool cliff[3];
	bool wall[3];

	bool IsOnCliff(){
		return cliff[0]||cliff[1]||cliff[2];
	}


	void Process(){
		if(cliff[0] || cliff[1] || cliff[2]){
			SetSpeed(-0.1, 0);
		}
	}
	void SetSpeed(float lin, float rot){

		//Security
		if(ctrl->IsOnCliff() && lin>0){
			std_msgs::String msg;
			msg.data = "play /usr/share/sounds/gnome/default/alerts/bark.ogg &";
			pub_shellcmd.publish(msg);
			return;
		}

		geometry_msgs::Twist msg;
		msg.linear.x = lin;
		msg.angular.x = rot;
		pub_velocity.publish(msg);
	}

	void Say(const string& text){
		cout<<"Turtlebot says: "<<text<<endl;
		string cmd = "echo \"" + text + "\" | espeak -s 160 -p 100&";
		std_msgs::String msg;
		msg.data = cmd.c_str();
		pub_shellcmd.publish(msg);
	}

	enum color{
		BLACK=0, GREEN=1, ORANGE=2, RED=3
	};
	void SetLed(int nLed, color color){
		kobuki_msgs::Led msg;
		msg.value = (uint8_t)color;

		if(nLed==1)
			pub_led1.publish(msg);
		else if(nLed==2)
			pub_led2.publish(msg);
		else
			ROS_INFO("LED inconnue");
	}


private:

};




void buttonCallback(const kobuki_msgs::ButtonEventConstPtr msg){
  	if(msg->button == kobuki_msgs::ButtonEvent::Button0){
		if(msg->state == kobuki_msgs::ButtonEvent::RELEASED)
			ctrl->SetLed(1, Controller::color::BLACK);
		else if(msg->state == kobuki_msgs::ButtonEvent::PRESSED)
			ctrl->SetLed(1, Controller::color::ORANGE);
	}
	else if(msg->button == kobuki_msgs::ButtonEvent::Button1){
		if(msg->state == kobuki_msgs::ButtonEvent::PRESSED){
			ctrl->SetSpeed(0.1,0);
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
	ctrl->cliff[msg->sensor] = msg->state;

	if(msg->state == kobuki_msgs::CliffEvent::CLIFF){
		std_msgs::String msg;
		msg.data = "play ~/nonono.ogg &";
		pub_shellcmd.publish(msg);
	}
}
void bumperCallback(const kobuki_msgs::BumperEventConstPtr msg){
	ctrl->wall[msg->bumper] = msg->state;

	if(msg->state == 1){
		ctrl->Say("Ouch");
	}
}


int main(int argc, char **argv){
	ros::init(argc, argv, "itb_main");
	node = new ros::NodeHandle;

	//Subscribe to sensors
	sub_bumper = node->subscribe("/mobile_base/events/bumper", 1000, bumperCallback);
	sub_button = node->subscribe("/mobile_base/events/button", 1000, buttonCallback);
	sub_cliff = node->subscribe("/mobile_base/events/cliff", 1000, cliffCallback);

	pub_velocity = node->advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
	pub_led1 = node->advertise<kobuki_msgs::Led>("/mobile_base/commands/led1", 1000);
	pub_led2 = node->advertise<kobuki_msgs::Led>("/mobile_base/commands/led2", 1000);
	pub_shellcmd = node->advertise<std_msgs::String>("itb_shellcmd_topic", 1000);

	usleep(1000000);

	ctrl = new Controller();

	ros::Rate loopRate(10);
	while(ros::ok()){
		ctrl->Process();

		ros::spinOnce();
		loopRate.sleep();
	}
	

	return 0;
}

