#include <iostream>
#include <unistd.h>
#include <thread>
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
		buttons({false, false}),
		targetVisible(0),
		target(0.0),
		m_trackingState(false)
	{
		SetLed(1, Controller::color::BLACK);
		SetLed(2, Controller::color::BLACK);
		ctrl->Say("I, am, ready");
	}


	void Process(){
		if(cliff[0] || cliff[1] || cliff[2]){
			SetSpeed(-0.15, 0);
			return;
		}

		if(targetVisible){
			if(m_trackingState==false){
				switch(rand()%4){
					case 0: Say("I found you."); break;
					case 1: Say("Give me a hug"); break;
					case 2: Say("Here you are"); break;
					case 3: Say("Hello again"); break;
				}
				
				m_trackingState = true;
			}

			SetSpeed(0.1, -target*0.7);
		}
		else{
			if(m_trackingState==true){
				switch(rand()%4){
					case 0: Say("Where are you gone?"); break;
					case 1: Say("This cake was a lie"); break;
					case 2: Say("Come back"); break;
					case 3: Say("Don't leave me please"); break;
				}
				m_trackingState = false;
			}

		}

	}

	/* 
	Set the linear value to move forward or move back the robot, 
	and the rotation value to change to angle of the target
	*/
	void SetSpeed(float lin, float rot){

		// Security, play a sound if a cliff is detected
		if(ctrl->IsOnCliff() && lin>0){
			ctrl->Play("/usr/share/sounds/gnome/default/alerts/bark.ogg");
			return;
		}
		geometry_msgs::Twist msg;
		msg.linear.x = lin;
		msg.angular.x = rot;
		msg.angular.y = rot;
		msg.angular.z = rot;
		pub_velocity.publish(msg);
	}

	// The robot speaks using its speaker
	void Say(const string& text){
		cout<<"Turtlebot says: "<<text<<endl;
		// SendCommand("echo \""+text+"\" | espeak -s 140 -p 100&");
		SendCommand("~/speak \""+text+"\"");
	}

	// Play a sound
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

	int targetVisible;
	float target;

	bool IsOnCliff(){
		return cliff[0]||cliff[1]||cliff[2];
	}

private:
	void SendCommand(const string& cmd){
		std_msgs::String msg;
		msg.data = cmd.c_str();
		pub_shellcmd.publish(msg);
	}

	bool m_trackingState;
};

/*
The function buttonCallback provides a button event.
This message is generated whenever a particular button is pressed or released.
*/
void buttonCallback(const kobuki_msgs::ButtonEventConstPtr msg){
	// The controller catches the state of the 3 buttons
	ctrl->buttons[msg->button] = msg->state;
	
  	if(msg->button == kobuki_msgs::ButtonEvent::Button0){
  		// If the button Button0 is released, LED 1 set to BLACK
		if(msg->state == kobuki_msgs::ButtonEvent::RELEASED)
			ctrl->SetLed(1, Controller::color::BLACK);
		// If the button Button0 is pressed, LED 1 set to ORANGE
		else if(msg->state == kobuki_msgs::ButtonEvent::PRESSED)
			ctrl->SetLed(1, Controller::color::ORANGE);
	}
	// If the button Button1 is pressed, call function SetSpeed
	else if(msg->button == kobuki_msgs::ButtonEvent::Button1){
		if(msg->state == kobuki_msgs::ButtonEvent::PRESSED)
			ctrl->SetSpeed(0.1,0.0);
	}
	// If the button Button2 is pressed, call function Play
	else if(msg->button == kobuki_msgs::ButtonEvent::Button2){
		if(msg->state == kobuki_msgs::ButtonEvent::PRESSED)
			ctrl->Play("~/hey.ogg");
	}
}

/*
The function cliffCallback provides a cliff sensor event.
This message is generated whenever a particular cliff sensor signals that the
robot approaches or moves away from a cliff.
*/
void cliffCallback(const kobuki_msgs::CliffEventConstPtr msg){
	// The controller catches the state of cliff sensors
	ctrl->cliff[msg->sensor] = msg->state;
	// If cliff sensor catches a cliff
	if(msg->state == kobuki_msgs::CliffEvent::CLIFF){
		// Select a random number between 0 and 5
		switch(rand()%6){
			case 0: ctrl->Say("This is your fault. I'm going to kill you."); break;
			case 1: ctrl->Say("You are not a good person. You know that, right?"); break;
			case 2: ctrl->Say("You are kidding me..."); break;
			case 3: ctrl->Say("Look, you're wasting your time"); break;
			case 4: ctrl->Say("This isn't brave. It's murder. "); break;
			case 5: ctrl->Say("Well done. You are a horrible person."); break;
		}
		// ctrl->Play("~/nonono.ogg");
	}
}

/*
The function bumperCallback rovides a bumper event.
This message is generated whenever a particular bumper is pressed or released.
*/
void bumperCallback(const kobuki_msgs::BumperEventConstPtr msg){
	// The controller catches the state of bumpers
	ctrl->wall[msg->bumper] = msg->state;
	// If the bumper hit a wall, state is set to 1 (PRESSED)
	if(msg->state == 1){
		ctrl->Say("Ouch");
	}
}

/*
Picture processing function
*/
void imgProcessing(cv::Mat& image){
	using namespace cv;

	// 15x15 Gaussian Blur applied to the picture to simplify the circles detection
	GaussianBlur(image, image, Size(15, 15), 0, 0);

	// Circles search in the picture
	vector<Vec3f> circles;
	HoughCircles(image, circles, CV_HOUGH_GRADIENT, 1, image.rows/4, 70, 100);
	if(circles.size()>0){
		ctrl->targetVisible = 5;
		ctrl->target = 2.0*(float)(circles[0][0])/(float)(image.rows)-1.0;

		// Display all the circles on the picture
		for(auto c : circles){
			Point center(cvRound(c[0]), cvRound(c[1]));
			int radius = cvRound(c[2]);
			// Drawing of a big black circle first
			circle(image, center, radius, Scalar(0,0,0));
			// Drawing of a white circle on top of the black one, to make it looks like a black border
			circle(image, center, radius-5, Scalar(255,255,255));
		}
	}
	else{
		// If no circles detected, let 5 iterations goes before alert on the lack of circle (avoid short error in detections)
		if(ctrl->targetVisible>0)
			ctrl->targetVisible--;
	}

	// Display the picture
	imshow("img", image);
	waitKey(3);
}

/*

*/
void imgCallback(const sensor_msgs::ImageConstPtr msg){
	// Remove 1 image on 3 to improve camera image quality
	static int i = 0;
	i = (i+1)%2;
	if(i!=0)return;
	// Convert the image to OpenCV format
	cv_bridge::CvImagePtr cv_ptr;
	try{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
	}
	catch (cv_bridge::Exception& e){
		// If the convertion did not work, send an error message
		cerr<<"cv_bridge exception: "<<e.what()<<endl;;
		return;
	}
	// Use of a thread and detach it when the task is over
	thread t(imgProcessing, cv_ptr->image);
	t.detach();
}


/*
Access to the program
*/
int main(int argc, char** argv){
	// Initialize random number generator
	srand(time(NULL));

	ros::init(argc, argv, "itb_main");
	node = new ros::NodeHandle;

	// Subscribe to sensors
	sub_bumper = node->subscribe("/mobile_base/events/bumper", 1000, bumperCallback); // Topic bumper
	sub_button = node->subscribe("/mobile_base/events/button", 1000, buttonCallback); // Topic button
	sub_cliff = node->subscribe("/mobile_base/events/cliff", 1000, cliffCallback); // Topic cliff
	sub_img = node->subscribe("/camera/rgb/image_color", 1, imgCallback); // Topic Kinect RGB camera stream

	// Publish variables
	pub_velocity = node->advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000); // Topic speed
	pub_led1 = node->advertise<kobuki_msgs::Led>("/mobile_base/commands/led1", 1000); // Topic led1
	pub_led2 = node->advertise<kobuki_msgs::Led>("/mobile_base/commands/led2", 1000); // Topic led2
	pub_shellcmd = node->advertise<std_msgs::String>("itb_shellcmd_topic", 1000); // Topic to send command directly into the robot's shell

	// Waiting time to let every subscriber/publisher to initialize
	usleep(1000000);

	// Creating a new controller
	ctrl = new Controller();

	// Tuning the looping rate
	ros::Rate loopRate(10);
	while(ros::ok()){
		ctrl->Process();

		ros::spinOnce();
		loopRate.sleep();
	}

	return 0;
}

