#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"
#include "ctre/phoenix/cci/PDP_CCI.h"
#include "ctre/phoenix/cci/CCI.h"
#include "ctre/phoenix/ErrorCode.h"
#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <SDL2/SDL.h>
#include <unistd.h>
#include <math.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/Joy.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"

#include <ctime>


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

//ctre::phoenix::motorcontrol::can::TalonSRX intakeArmMotor(9);

//ctre::phoenix::motorcontrol::can::TalonSRX intakeWheelMotor(10);

//ctre::phoenix::sensors::CANCoder intakeArmEncoder(5, interface);

double pdpVoltage = 0.0;
int pdpCurrentsFilled = 0;
double* pdpCurrents = {};

double * voltagePtr = &pdpVoltage;
int * currentsFilledPtr = &pdpCurrentsFilled;

void initializeIntakeMotors(){
	//intakeArmMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 9);
}

void initializeDriveMotors(){
	//PID config
	float kP = 0.1;
	float kI = 0.0;
	float kD = 0.0;

	frontRightDrive.Config_kP(0, kP);
	frontRightDrive.Config_kI(0, kI);
	frontRightDrive.Config_kD(0, kD);

	frontLeftDrive.Config_kP(0, kP);
	frontLeftDrive.Config_kI(0, kI);
	frontLeftDrive.Config_kD(0, kD);

	backLeftDrive.Config_kP(0, kP);
	backLeftDrive.Config_kI(0, kI);
	backLeftDrive.Config_kD(0, kD);

	backRightDrive.Config_kP(0, kP);
	backRightDrive.Config_kI(0, kI);
	backRightDrive.Config_kD(0, kD);

	//zero drive motors
	while (frontRightDrive.GetSelectedSensorPosition() != 0.0){
		frontRightDrive.SetSelectedSensorPosition(0.0);
	}
	while (frontLeftDrive.GetSelectedSensorPosition() != 0.0){
		frontLeftDrive.SetSelectedSensorPosition(0.0);
	}
	while (backLeftDrive.GetSelectedSensorPosition() != 0.0){
		backLeftDrive.SetSelectedSensorPosition(0.0);
	}
	while (backRightDrive.GetSelectedSensorPosition() != 0.0){
		backRightDrive.SetSelectedSensorPosition(0.0);
	}

	frontRightDrive.Set(ControlMode::PercentOutput, 0.0);
	frontLeftDrive.Set(ControlMode::PercentOutput, 0.0);
	backLeftDrive.Set(ControlMode::PercentOutput, 0.0);
	backRightDrive.Set(ControlMode::Velocity, 0.0);
}

//maximum rpm of drive motors
double maxVelocity = 6380.0;

//distance between right and left wheels (meters)
//16 and 11/16 inches
double wheelSpacing = 0.4238625;

//wheel circumference (meters)
double wheelCircumference = 0.608447957;

//gear ratio of the drive motors
double motorToWheelRatio = 12.0;

//cancoder resolutions
double cancoderTicksPerRevolution = 4096.0;

//whether joystick is connected
bool haveJoystick = true;

//convert motor ticks to meters
double getDistanceFromTicks(double ticks){
	return (ticks / 2048.0) * wheelCircumference / motorToWheelRatio;
}

double getTicksPer100MSFromVelocity(double velocity){
	return (((velocity / wheelCircumference) * 2048.0) / 10.0) * 12.0;
}

double getVelocityFromTicksPer100MS(double ticksPer100MS){
	return (((ticksPer100MS * 10.0) / 2048.0) * wheelCircumference) / 12.0;
}

//pose object for returning odometry info
class Pose {
	public:
	double x, y, angle;
};

class Odometry {
	public:

	//mirv pose
	double x = 0.0;
	double y = 0.0;
	double angle = M_PI / 2.0;

	//radius of circular path of travel
	double r = 0.0;
	//angular displacement of mirv around circular path of travel
	double deltaTheta = 0.0;

	//tick position from previous update
	double prevLeftTicks = 0.0;
	double prevRightTicks = 0.0;

	void update(){
		double leftTicks = frontLeftDrive.GetSelectedSensorPosition();
		double rightTicks = -frontRightDrive.GetSelectedSensorPosition();

		//calculate how many ticks the motor moved by
		double leftDisplacement = getDistanceFromTicks(leftTicks - prevLeftTicks);
		double rightDisplacement = getDistanceFromTicks(rightTicks - prevRightTicks);
		
		if (leftDisplacement == rightDisplacement){
			//radius calculation edge case
			double displacement = rightDisplacement;

			x += displacement * cos(angle);
			y += displacement * sin(angle);
		} else {
			//calculate radius and change in theta
			r = abs((wheelSpacing * (rightDisplacement + leftDisplacement)) / (2 * (rightDisplacement - leftDisplacement)));
			
			deltaTheta = abs(rightDisplacement - leftDisplacement) / wheelSpacing;
			if (leftDisplacement > rightDisplacement){
				angle -= deltaTheta;
			} else {
				angle += deltaTheta;
			}

			//update pose
			x += (r * cos(deltaTheta) * sin(angle)) + (r * cos(angle) * sin(deltaTheta)) - (r * sin(angle));
			y += (r * sin(deltaTheta) * sin(angle)) - (r * cos(angle) * cos(deltaTheta)) + (r * cos(angle));
		}

		prevLeftTicks = leftTicks;
		prevRightTicks = rightTicks;
	}

	Pose getPose(){
		Pose p;
		p.x = x;
		p.y = y;
		p.angle = angle;
		return p;
	}

	double getX(){
		return x;
	}
	double getY(){
		return y;
	}
	double getAngle(){
		return angle;
	}
};

Odometry odometry;

class Publisher {
	public:

	ros::Publisher batteryVoltagePub;
	ros::Publisher encoderOdometryPub;
	ros::Publisher intakeStatusPub;
	ros::Publisher encoderVelocityPub;

	void publishVoltage(){
		std_msgs::Float64 voltage;
		auto ExportError = c_PDP_GetValues(0, voltagePtr, pdpCurrents, 0, currentsFilledPtr);
		voltage.data = pdpVoltage;
		batteryVoltagePub.publish(voltage);
	}

	void publishOdometry(){
		std_msgs::Float64MultiArray pose;
		pose.data.push_back(odometry.getX());
		pose.data.push_back(odometry.getY());
		pose.data.push_back(odometry.getAngle());
		encoderOdometryPub.publish(pose);
	}

	void publishIntakeStatus(std_msgs::String status){
		intakeStatusPub.publish(status);
	}

	void publishEncoderVelocity(){
		publishEncoderVelocity.publish()
	}
};

Publisher publisher;

class Intake {
	

	public:
	std::string mode = "disable";
	std::string modes[4] = {"disable", "reset", "intake", "store"};
	bool haveReported = false;

	// void update(){
	// 	//not moving at all
	// 	if (mode == "disable"){
	// 		intakeArmMotor.Set(ControlMode::PercentOutput, 0.0);
	// 	}
		
	// 	//move to upright and zero
	// 	if (mode == "reset"){
	// 		if (intakeArmMotor.GetSensorCollection().IsRevLimitSwitchClosed() == 1){
	// 			intakeArmMotor.Set(ControlMode::PercentOutput, 0.0);
	// 			intakeArmEncoder.SetPosition(0.0);
	// 			if (!haveReported){
	// 				std_msgs::String status;
	// 				status.data = "reset finished";
	// 				publisher.publishIntakeStatus(status);
	// 			}
	// 			haveReported = true;
	// 		} else {
	// 			intakeArmMotor.Set(ControlMode::PercentOutput, 0.2);
	// 		}
	// 	}

	// 	if (mode == "intake"){
	// 		intakeWheelMotor.Set(ControlMode::PercentOutput, 0.2);
			
	// 	}
	// }

	void setMode(std::string cmd){

		//check if command is valid
		//set if it is
		bool isValid = false;
		for (int i = 0; i < sizeof(modes); i ++){
			if (modes[i] == cmd){
				isValid = true;
			}
		}
		if (isValid){
			mode = cmd;
			haveReported = false;
		}
	}
};

Intake intake;

// Set speed of individual motors
void setDriveCallback(const std_msgs::Float32MultiArray::ConstPtr& msg){
	ROS_INFO("I heard: [%f]", msg->data[0]);

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
}

void velocityDriveCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){

	double leftVelocity = msg->data[0];
	double rightVelocity = msg->data[1];

	double leftTicksPer100MS = getTicksPer100MSFromVelocity(leftVelocity);
	double rightTicksPer100MS = -getTicksPer100MSFromVelocity(rightVelocity);

	frontRightDrive.Set(ControlMode::Velocity, rightTicksPer100MS);
	backRightDrive.Set(ControlMode::Velocity, rightTicksPer100MS);

	frontLeftDrive.Set(ControlMode::Velocity, leftTicksPer100MS);
	backLeftDrive.Set(ControlMode::Velocity, leftTicksPer100MS);
}

void diagnosticCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr& statusMsg){
	/*
	Check whether joystick is connected
	If not, stop drive motors
	*/
	int level = statusMsg->status[0].level;
	if (level == 0){
		haveJoystick = true;
	} else if (level == 2){
		haveJoystick = false;
		ROS_INFO("Jostick DISCONNECTED");
		frontRightDrive.Set(ControlMode::PercentOutput, 0.0);
		frontLeftDrive.Set(ControlMode::PercentOutput, 0.0);
		backLeftDrive.Set(ControlMode::PercentOutput, 0.0);
		backRightDrive.Set(ControlMode::PercentOutput, 0.0);
	}
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy){

	double drive = joy->axes[1];
	double turn = joy->axes[3];
	double rightMotorPower = -drive - turn;
	double leftMotorPower = drive - turn;

	//Turn motor percent to percent of maximum velocity
	double rightMotorRPM = rightMotorPower * maxVelocity;
	double leftMotorRPM = leftMotorPower * maxVelocity;
	
	//Convert percent of maximum velocity to tic per 100 ms
	double rightMotorTP100MS = (rightMotorRPM / 600.0) * 2048.0;
	double leftMotorTP100MS = (leftMotorRPM / 600.0) * 2048.0;

	if (haveJoystick){
		frontRightDrive.Set(ControlMode::Velocity, rightMotorTP100MS);
		frontLeftDrive.Set(ControlMode::Velocity, leftMotorTP100MS);
		backLeftDrive.Set(ControlMode::Velocity, leftMotorTP100MS);
		backRightDrive.Set(ControlMode::Velocity, rightMotorTP100MS);
 	}
}

void intakeCommandCallback(const std_msgs::String::ConstPtr& cmd){
	intake.setMode(cmd->data);
}

/** simple wrapper for code cleanup */
void sleepApp(int ms)
{
	std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "canbus");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("drive", 10, setDriveCallback);
	ros::Subscriber joySub = n.subscribe("joy", 10, joyCallback);
	ros::Subscriber diagnosticSub = n.subscribe("diagnostics", 10, diagnosticCallback);
	ros::Subscriber velocityDriveSub = n.subscribe("VelocityDrive", 10, velocityDriveCallback);
	ros::Subscriber intakeCommandSub = n.subscribe("intake/command", 10, intakeCommandCallback);

	publisher.batteryVoltagePub = n.advertise<std_msgs::Float64>("battery/voltage", 10);
	ros::Timer batteryVoltageTimer = n.createTimer(ros::Duration(1), std::bind(&Publisher::publishVoltage, publisher));

	publisher.encoderOdometryPub = n.advertise<std_msgs::Float64MultiArray>("odometry/encoder", 10);
	ros::Timer encoderOdometryTimer = n.createTimer(ros::Duration(1.0 / 50.0), std::bind(&Publisher::publishOdometry, publisher));

	publisher.encoderVelocityPub = n.advertise<std_msgs::Float64MultiArray>("encoder/velocities", 10);

	publisher.intakeStatusPub = n.advertise<std_msgs::String>("intake/status", 10);

	

	initializeDriveMotors();
	initializeIntakeMotors();

	while(ros::ok()){
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100);

		odometry.update();
		//intake.update();

		ros::spinOnce();
	}
  	
	return 0;
}