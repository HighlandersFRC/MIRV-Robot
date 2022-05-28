#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"
#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <SDL2/SDL.h>
#include <unistd.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/Joy.h"


using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;
using namespace std;

/* make some talons for drive train */
std::string interface = "can0";

TalonFX frontRightDrive(1, interface); 
TalonFX frontLeftDrive(2, interface); 
TalonFX backLeftDrive(3, interface); 
TalonFX backRightDrive(4, interface);


// Set speed of individual motors
void setDriveCallback(const std_msgs::Float32MultiArray::ConstPtr& msg){
	ROS_INFO("I heard: [%f]", msg->data[0]);

	//int len = *(&msg->data + 1) - &msg->data;
	int len = 2;
	if(len ==2){
		int talonID = (int)msg->data[0];
		float motorSpeed = msg->data[1];
		if(talonID == 1){
			frontRightDrive.Set(ControlMode::PercentOutput, motorSpeed);
		}
		else if (talonID == 2){
			frontLeftDrive.Set(ControlMode::PercentOutput, motorSpeed);

		}
		else if (talonID == 3){
			backLeftDrive.Set(ControlMode::PercentOutput, motorSpeed);

		}
		else if (talonID == 4){
			backRightDrive.Set(ControlMode::PercentOutput, motorSpeed);
		
		}
	}
	//ROS_INFO(msg->data);
	

	//.Set(ControlMode::PercentOutput, 0.1);
	
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
	float drive = joy->axes[1];
	float turn = joy->axes[3];
	float rightMotorPower = drive + turn;
	float leftMotorPower = -drive + turn;



	ROS_INFO("Drive: [%f], Turn: [%f], Right Motors: [%f], Left Motors: [%f]", drive, turn, rightMotorPower, leftMotorPower);
	

	frontRightDrive.Set(ControlMode::PercentOutput, rightMotorPower);
	frontLeftDrive.Set(ControlMode::PercentOutput, leftMotorPower);
	backLeftDrive.Set(ControlMode::PercentOutput, leftMotorPower);
	backRightDrive.Set(ControlMode::PercentOutput, rightMotorPower);

}


/** simple wrapper for code cleanup */
void sleepApp(int ms)
{
	std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "drive");
  	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("drive", 10, setDriveCallback);
	ros::Subscriber joySub = n.subscribe("joy", 10, joyCallback);
  	//ros::spin();

	while(ros::ok()){
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100);
		ros::spinOnce();
		
	}
	//while (true) {
	//	ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100);
	//	
	//	sleepApp(20);
	//}
	
	
	
  	
	return 0;
}
