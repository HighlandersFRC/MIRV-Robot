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

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;
using namespace std;

/* make some talons for drive train */
std::string interface = "can0";

TalonFX falcon(0, interface); //Use the default interface (can0)

/** simple wrapper for code cleanup */
void sleepApp(int ms)
{
	std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

int main(int argc, char **argv) {	
	while (true) {
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100);
		falcon.Set(ControlMode::PercentOutput, 0.1);
		sleepApp(20);
	}
	
	
	//ros::init(argc, argv, "drive");
  	//ros::NodeHandle n;
  	//ros::spin();
  	
	return 0;
}
