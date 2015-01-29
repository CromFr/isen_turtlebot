#include <iostream>
#include <unistd.h>
using namespace std;

#include <ros/ros.h>
//Messages
#include <std_msgs/String.h>
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/ButtonEvent.h>
#include <kobuki_msgs/CliffEvent.h>
#include <kobuki_msgs/Led.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
//OpenCV
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>

class Controller;
Controller* ctrl;

ros::NodeHandle* node;
ros::Subscriber 
	sub_bumper,
	sub_button,
	sub_cliff,
	sub_img;
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
		wall({false, false, false}),
		buttons({false, false})
	{
		SetLed(1, Controller::color::BLACK);
		SetLed(2, Controller::color::BLACK);
		ctrl->Say("I, am, ready");
	}


	void Process(){
		if(cliff[0] || cliff[1] || cliff[2]){
			SetSpeed(-0.1, 0);
		}
	}





	void SetSpeed(float lin, float rot){

		//Security
		if(ctrl->IsOnCliff() && lin>0){
			ctrl->Play("/usr/share/sounds/gnome/default/alerts/bark.ogg");
			return;
		}

		geometry_msgs::Twist msg;
		msg.linear.x = lin;
		msg.angular.x = rot;
		pub_velocity.publish(msg);
	}

	void Say(const string& text){
		cout<<"Turtlebot says: "<<text<<endl;
		SendCommand("echo \""+text+"\" | espeak -s 160 -p 100&");
	}

	void Play(const string& soundFile){
		SendCommand("play -v 0.5 "+soundFile+"&");
	}

	enum class color{
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

	bool cliff[3];
	bool wall[3];
	bool buttons[2];

	bool IsOnCliff(){
		return cliff[0]||cliff[1]||cliff[2];
	}

private:
	void SendCommand(const string& cmd){
		std_msgs::String msg;
		msg.data = cmd.c_str();
		pub_shellcmd.publish(msg);
	}

};




void buttonCallback(const kobuki_msgs::ButtonEventConstPtr msg){
	ctrl->buttons[msg->button] = msg->state;

  	if(msg->button == kobuki_msgs::ButtonEvent::Button0){
		if(msg->state == kobuki_msgs::ButtonEvent::RELEASED)
			ctrl->SetLed(1, Controller::color::BLACK);
		else if(msg->state == kobuki_msgs::ButtonEvent::PRESSED)
			ctrl->SetLed(1, Controller::color::ORANGE);
	}
	else if(msg->button == kobuki_msgs::ButtonEvent::Button1){
		if(msg->state == kobuki_msgs::ButtonEvent::PRESSED)
			ctrl->SetSpeed(0.1,0);
	}
	else if(msg->button == kobuki_msgs::ButtonEvent::Button2){
		if(msg->state == kobuki_msgs::ButtonEvent::PRESSED)
			ctrl->Play("~/hey.ogg");
	}
}

void cliffCallback(const kobuki_msgs::CliffEventConstPtr msg){
	ctrl->cliff[msg->sensor] = msg->state;

	if(msg->state == kobuki_msgs::CliffEvent::CLIFF)
		ctrl->Play("~/nonono.ogg");
}
void bumperCallback(const kobuki_msgs::BumperEventConstPtr msg){
	ctrl->wall[msg->bumper] = msg->state;

	if(msg->state == 1){
		ctrl->Say("Ouch");
	}
}
void imgCallback(const sensor_msgs::ImageConstPtr msg){
	cv_bridge::CvImagePtr cv_ptr;
	try{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e){
		cerr<<"cv_bridge exception: "<<e.what()<<endl;;
		return;
	}
	//OpenCV Processing:
	//===================
	using namespace cv;

	cvtColor(cv_ptr->image, cv_ptr->image,CV_RGB2GRAY);

	vector<Vec3f> circles;
	HoughCircles(cv_ptr->image, circles, CV_HOUGH_GRADIENT, 4, cv_ptr->image.rows/4, 200, 200 );
	for( size_t i = 0; i < circles.size(); i++ ) {
		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		int radius = cvRound(circles[i][2]);
		circle(cv_ptr->image, center, radius, Scalar(rand()%255, rand()%255, rand()%255), -1, 8, 0 );
	}

	cv::imshow("img", cv_ptr->image);
	cv::waitKey(3);
}


int main(int argc, char** argv){
	ros::init(argc, argv, "itb_main");
	node = new ros::NodeHandle;

	//Subscribe to sensors
	sub_bumper = node->subscribe("/mobile_base/events/bumper", 1000, bumperCallback);
	sub_button = node->subscribe("/mobile_base/events/button", 1000, buttonCallback);
	sub_cliff = node->subscribe("/mobile_base/events/cliff", 1000, cliffCallback);
	sub_img = node->subscribe("/camera/rgb/image_color", 1000, imgCallback);

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

