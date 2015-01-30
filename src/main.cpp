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
bool setColor = false;
bool hasBeenSetColor = false;
void mesureColorAverage(cv_bridge::CvImagePtr& cv_ptr,vector<int>& color);
void giveTheObjectPosition(cv_bridge::CvImagePtr& cv_ptr, std::vector<int>& color, std::vector<int>& position, int sections, float threshold);

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
		targetVisible(false),
		target(0.0)
	{
		SetLed(1, Controller::color::BLACK);
		SetLed(2, Controller::color::BLACK);
		ctrl->Say("I, am, ready");
	}


	void Process(){
		if(cliff[0] || cliff[1] || cliff[2]){
			SetSpeed(-0.1, 0);
		}

		//cout<<targetVisible<<"=> "<<target<<endl;
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

	bool targetVisible;
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
			setColor = true;
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
	static int i = 0;
	i = (i+1)%3;
	if(i!=0)return;

	std::vector<int> color;
	std::vector<int> position;//position in pixel of the object
	int sections = 160;//we divide the picture in 160 parts
	float threshold = 0.065;//detection threshold in % of 255, ex : 0.05 => 0.05*255

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

	if(!hasBeenSetColor){
		//we define and draw the rect
		cv::rectangle(cv_ptr->image,cvPoint(290,210),cvPoint(350,270),CV_RGB(0,255,0),5,8);

		if (setColor)
		{
			mesureColorAverage(cv_ptr, color);
			setColor = false;
			hasBeenSetColor = true;
		}
	}else{
		cout << "coucou" << endl;
		giveTheObjectPosition(cv_ptr, color, position, sections, threshold);
		cout << "coucou2" << endl;
		cv::rectangle(cv_ptr->image,cvPoint(position[0]-50,position[1]-50),cvPoint(position[0]+50,position[1]+50),CV_RGB(0,255,0),5,8);
		cout << "coucou3" << endl;
	}
	cv::imshow("img", cv_ptr->image);
	cv::waitKey(3);
}

void mesureColorAverage(cv_bridge::CvImagePtr& cv_ptr,vector<int>& color)
{
	//we define an array which contain the average value of rgb colors of the rectangle
	int averageColor[3]={0,0,0};
	int count=0;

	for(int x=298; x<=342;x++){
		for(int y=218;y<=268;y++){
			averageColor[0] += cv_ptr->image.at<cv::Vec3b>(y,x)[0];
			averageColor[1] += cv_ptr->image.at<cv::Vec3b>(y,x)[1];
			averageColor[2] += cv_ptr->image.at<cv::Vec3b>(y,x)[2];
			count++;
		}
	}
	for(size_t i=0; i <3; i++){
		color.push_back((int)((double)averageColor[i]/(double)count));
	}
	cout << "Red : " << color[2] << " - Green : " << color[1] << " - Blue : " << color[0] << endl;
}

void giveTheObjectPosition(cv_bridge::CvImagePtr& cv_ptr, std::vector<int>& color, std::vector<int>& position, int sections, float threshold)
{	
	int pixelPerSectionsX = 640/sections;
	int pixelPerSectionsY = 480/sections;
	int startX = 0;
	int endX = 0;
	int startY = 0;
	int endY = 0;
	int averageColor[3] = {0,0,0};
	int count = 0;
	int testR = 0;
	int testG = 0;
	int testB = 0;
	bool isColor[sections][sections];
	int countColor = 0;
	int countMax = 0;

	cout << "hello" << endl;
	position.clear();   // clear last position
	
	cout << "hello2" << endl;
	for (int i=0; i<sections; i++){
		for (int j=0; j<sections; j++){
			isColor[i][j] = 0;
		}
	}
	
	cout << "hello3" << endl;
	for(int boxY=0; boxY<sections; boxY++)
	{
		startY = pixelPerSectionsY * boxY;
		endY = pixelPerSectionsY * (boxY+1); 
		//we start to analyse the x orientation
		for(int boxX=0; boxX<sections; boxX++)
		{
			testR = 0; testG = 0; testB = 0;
			count = 0;
			startX = pixelPerSectionsX * boxX;
			endX = pixelPerSectionsX * (boxX+1); 
			for(int x=startX; x<endX; x++)
			{
				for(int y=startY; y<endY; y++)
				{
					averageColor[0] += cv_ptr->image.at<cv::Vec3b>(y,x)[0];
					averageColor[1] += cv_ptr->image.at<cv::Vec3b>(y,x)[1];
					averageColor[2] += cv_ptr->image.at<cv::Vec3b>(y,x)[2];
					count++;
				}
			}
	cout << "hello4" << endl;
			//now we calcul the average and we compare it with the threshold
			averageColor[0] = averageColor[0]/count;
			averageColor[1] = averageColor[1]/count;
			averageColor[2] = averageColor[2]/count;
			
			if(averageColor[2] <= (color[2]+(int)(threshold*255)) && averageColor[2] >= (color[2]-(int)(threshold*255)))
				testR = 1;
			if(averageColor[1] <= (color[1]+(int)(threshold*255)) && averageColor[1] >= (color[1]-(int)(threshold*255)))
				testG = 1;
			if(averageColor[0] <= (color[0]+(int)(threshold*255)) && averageColor[0] >= (color[0]-(int)(threshold*255)))
				testB = 1;
			if(testR && testG && testB)//the color of this box correspond to the object
			{
				isColor[boxX][boxY] = 1;
			}
				
		}
		//we search for the maximums of the x and y sections in the tab isColor
		
	cout << "hello5" << endl;
		for(int x=0; x<sections; x++)
		{
			for(int y=0; y<sections; y++)
			{
				if(isColor[x][y])
				{
					countColor++;
				}
			}
			if(countColor>countMax)//we push and convert the position from sections to pixels
			{
				position.push_back(x*pixelPerSectionsX);
				//printf("push x : %d ", x*pixelPerSectionsX);
				countMax = countColor;
			}
			countColor = 0;

		}
	cout << "hello6" << endl;
		countMax = 0;
		for(int y=0; y<sections; y++)
		{
			for(int x=0; x<sections; x++)
			{
				if(isColor[x][y])
				{
					countColor++;
				}
			}
			if(countColor>countMax)//we push and convert the position from sections to pixels
			{
				position.push_back(y*pixelPerSectionsY);
				//printf("push y : %d ",y*pixelPerSectionsY);
				countMax = countColor;
			}
			countColor = 0;
		}
		
	}

}


int main(int argc, char** argv){
	ros::init(argc, argv, "itb_main");
	node = new ros::NodeHandle;

	//Subscribe to sensors
	sub_bumper = node->subscribe("/mobile_base/events/bumper", 1000, bumperCallback);
	sub_button = node->subscribe("/mobile_base/events/button", 1000, buttonCallback);
	sub_cliff = node->subscribe("/mobile_base/events/cliff", 1000, cliffCallback);
	sub_img = node->subscribe("/camera/rgb/image_color", 1, imgCallback);

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

