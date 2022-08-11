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
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include <time.h>

#include <ctime>


using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;
using namespace std;
double getTicksPer100MSFromVelocity(double velocity);
/* make some talons for drive train */
std::string interface = "can0";

TalonFX frontRightDrive(1, interface); 
TalonFX frontLeftDrive(2, interface); 
TalonFX backLeftDrive(3, interface); 
TalonFX backRightDrive(4, interface);

ctre::phoenix::motorcontrol::can::TalonSRX intakeArmMotor(11);
ctre::phoenix::motorcontrol::can::TalonSRX intakeWheelMotor(9);
ctre::phoenix::motorcontrol::can::TalonSRX leftConvMotor(10);
ctre::phoenix::motorcontrol::can::TalonSRX rightConvMotor(12);


double pdpVoltage = 0.0;
int pdpCurrentsFilled = 0;
double* pdpCurrents = {};

double * voltagePtr = &pdpVoltage;
int * currentsFilledPtr = &pdpCurrentsFilled;


double wheelBaseWidth = 0.0;
double wheelRadius = 0.0;

bool isDocked = false;

void initializeDriveMotors(){
	//PID config
	float kF = 0.025;
	float kP = 0.18;
	float kI = 0.000;
	float kD = 0.5;
	float maxAllowedError = getTicksPer100MSFromVelocity(0.05);

	frontRightDrive.ConfigFactoryDefault();
	frontLeftDrive.ConfigFactoryDefault();
	backLeftDrive.ConfigFactoryDefault();
	backRightDrive.ConfigFactoryDefault();

	frontRightDrive.ConfigAllowableClosedloopError(0, maxAllowedError);
	frontRightDrive.Config_kF(0, kF);
	frontRightDrive.Config_kP(0, kP);
	frontRightDrive.Config_kI(0, kI);
	frontRightDrive.Config_kD(0, kD);

	frontLeftDrive.ConfigAllowableClosedloopError(0, maxAllowedError);
	frontLeftDrive.Config_kF(0, kF);
	frontLeftDrive.Config_kP(0, kP);
	frontLeftDrive.Config_kI(0, kI);
	frontLeftDrive.Config_kD(0, kD);

	backLeftDrive.ConfigAllowableClosedloopError(0, maxAllowedError);
	backLeftDrive.Config_kF(0, kF);
	backLeftDrive.Config_kP(0, kP);
	backLeftDrive.Config_kI(0, kI);
	backLeftDrive.Config_kD(0, kD);

	backRightDrive.ConfigAllowableClosedloopError(0, maxAllowedError);
	backRightDrive.Config_kF(0, kF);
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
	backRightDrive.Set(ControlMode::PercentOutput, 0.0);
}

void initializeIntakeMotors(){
	intakeArmMotor.SetInverted(false);
	intakeArmMotor.ConfigReverseLimitSwitchSource(LimitSwitchSource_FeedbackConnector, LimitSwitchNormal_NormallyClosed);
	intakeArmMotor.ConfigForwardLimitSwitchSource(LimitSwitchSource_FeedbackConnector, LimitSwitchNormal_NormallyClosed);

	intakeWheelMotor.ConfigReverseLimitSwitchSource(LimitSwitchSource_FeedbackConnector, LimitSwitchNormal_NormallyOpen);
	intakeWheelMotor.ConfigForwardLimitSwitchSource(LimitSwitchSource_FeedbackConnector, LimitSwitchNormal_NormallyOpen);

	rightConvMotor.ConfigForwardLimitSwitchSource(LimitSwitchSource_FeedbackConnector, LimitSwitchNormal_NormallyOpen);
	rightConvMotor.ConfigReverseLimitSwitchSource(LimitSwitchSource_FeedbackConnector, LimitSwitchNormal_NormallyOpen);

	intakeWheelMotor.OverrideLimitSwitchesEnable(false);
	rightConvMotor.OverrideLimitSwitchesEnable(false);
	leftConvMotor.OverrideLimitSwitchesEnable(false);
}

//maximum rpm of drive motors
double maxVelocity = 6380.0;

//distance between right and left wheels (meters)
//16 and 11/16 inches
double wheelSpacing = 0.42038625;

//wheel circumference (meters)
double wheelCircumference = 0.59438;

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
	ros::Publisher encoderVelocityPub;
	ros::Publisher intakeLSPub;
	ros::Publisher touchSensorPub;

	void publishVoltage(){
		std_msgs::Float64 voltage;
		auto ExportError = c_PDP_GetValues(0, voltagePtr, pdpCurrents, 0, currentsFilledPtr);
		voltage.data = pdpVoltage;
		batteryVoltagePub.publish(voltage);
	}

	void publishEncoderVelocity(){
		sensor_msgs::JointState jointState;

		jointState.name = {
			"front_right_drive",
			"front_left_drive",
			"back_right_drive",
			"back_left_drive"
		};

		jointState.position = {
			frontRightDrive.GetSelectedSensorPosition(),
			frontLeftDrive.GetSelectedSensorPosition(),
			backRightDrive.GetSelectedSensorPosition(),
			backLeftDrive.GetSelectedSensorPosition(),

		};

		jointState.velocity  = {
			-getVelocityFromTicksPer100MS(frontRightDrive.GetSelectedSensorVelocity()),
			getVelocityFromTicksPer100MS(frontLeftDrive.GetSelectedSensorVelocity()),
			-getVelocityFromTicksPer100MS(backRightDrive.GetSelectedSensorVelocity()),
			getVelocityFromTicksPer100MS(backLeftDrive.GetSelectedSensorVelocity()),
		};

		encoderVelocityPub.publish(jointState);
	}

	void publishIntakeLimitSwitches(){
		std_msgs::Float64MultiArray limitSwitches;
		limitSwitches.data.push_back(intakeArmMotor.GetSensorCollection().IsFwdLimitSwitchClosed());
		limitSwitches.data.push_back(intakeArmMotor.GetSensorCollection().IsRevLimitSwitchClosed());
		limitSwitches.data.push_back(intakeWheelMotor.GetSensorCollection().IsFwdLimitSwitchClosed());
		limitSwitches.data.push_back(intakeWheelMotor.GetSensorCollection().IsRevLimitSwitchClosed());
		intakeLSPub.publish(limitSwitches);
	}

	void publishTouchSensor(){
		std_msgs::Float64MultiArray touchSensors;
		touchSensors.data.push_back(rightConvMotor.GetSensorCollection().IsFwdLimitSwitchClosed());
		touchSensors.data.push_back(rightConvMotor.GetSensorCollection().IsRevLimitSwitchClosed());
		touchSensorPub.publish(touchSensors);
	};
};

Publisher publisher;

class Intake {
	
	public:
	std::string mode = "disable";
	time_t startTime;
	double side = 1;
	//char* modes[4] = {"disable", "reset", "intake", "store"};

	//disable: all motors off
	//reset: move to upright position
	//intake: run wheels and move to downwards position
	//store: move to upwards position then reverse wheels
	//down: move intake to downwards position
	//mag_in: move magazine inwards
	//mag_out: move magazine outwards

	double intakeUpPercent = 0.8;
	double wheelIntakePercent = 0.6;

	//limit switches: fwd - up, rev - down

	void update(){

		//Disable if docked
		if (isDocked){
			mode = "disable";
		}

		// cout << " FwdB:";
		// cout << intakeWheelMotor.GetSensorCollection().IsFwdLimitSwitchClosed();
		// cout << " RevB:";
		// cout << intakeWheelMotor.GetSensorCollection().IsRevLimitSwitchClosed();
		// cout << " FwdS:";
		// cout << intakeArmMotor.GetSensorCollection().IsFwdLimitSwitchClosed();
		// cout << " RevS:";
		// cout << intakeArmMotor.GetSensorCollection().IsRevLimitSwitchClosed();

		//not moving at all
		if (mode == "disable"){
			intakeArmMotor.Set(ControlMode::PercentOutput, 0.0);
			intakeWheelMotor.Set(ControlMode::PercentOutput, 0.0);
			rightConvMotor.Set(ControlMode::PercentOutput, 0.0);
			leftConvMotor.Set(ControlMode::PercentOutput, 0.0);
		}
		
		//move to upright and zero
		if (mode == "reset"){
			intakeWheelMotor.Set(ControlMode::PercentOutput, 0.0);
			rightConvMotor.Set(ControlMode::PercentOutput, 0.0);
			leftConvMotor.Set(ControlMode::PercentOutput, 0.0);
			if (intakeArmMotor.GetSensorCollection().IsFwdLimitSwitchClosed() == 0){
				intakeArmMotor.Set(ControlMode::PercentOutput, 0.0);
			} else {
				intakeArmMotor.Set(ControlMode::PercentOutput, intakeUpPercent);
			}
		}

		if (mode == "intake"){
			// cout << "<";
			// cout << intakeArmMotor.GetSensorCollection().IsFwdLimitSwitchClosed();
			// cout << intakeArmMotor.GetSensorCollection().IsRevLimitSwitchClosed();
			// cout << ">";
			rightConvMotor.Set(ControlMode::PercentOutput, 0.0);
			leftConvMotor.Set(ControlMode::PercentOutput, 0.0);
			if (side > 0){
				if (intakeWheelMotor.GetSensorCollection().IsFwdLimitSwitchClosed() == 1){
					intakeWheelMotor.Set(ControlMode::PercentOutput, 0.0);
				} else {
					intakeWheelMotor.Set(ControlMode::PercentOutput, -wheelIntakePercent * side);
				}
			} else {
				if (intakeWheelMotor.GetSensorCollection().IsRevLimitSwitchClosed() == 1){
					intakeWheelMotor.Set(ControlMode::PercentOutput, 0.0);
				} else {
					intakeWheelMotor.Set(ControlMode::PercentOutput, -wheelIntakePercent * side);
				}
			}
			if (intakeArmMotor.GetSensorCollection().IsRevLimitSwitchClosed() == 0){
				intakeArmMotor.Set(ControlMode::PercentOutput, 0.0);
			} else {
				intakeArmMotor.Set(ControlMode::PercentOutput, -0.5);
			}
		}

		if (mode == "store"){
			if (intakeArmMotor.GetSensorCollection().IsFwdLimitSwitchClosed() == 0){
				intakeArmMotor.Set(ControlMode::PercentOutput, 0.0);
				intakeWheelMotor.Set(ControlMode::PercentOutput, 0.4 * side);
				if (side > 0){
					rightConvMotor.Set(ControlMode::PercentOutput, -0.4);
				} else {
					leftConvMotor.Set(ControlMode::PercentOutput, 0.4);
				}
				if (time(NULL) - startTime > 2){
					mode = "reset";
				}
			} else {
				intakeArmMotor.Set(ControlMode::PercentOutput, intakeUpPercent);
				intakeWheelMotor.Set(ControlMode::PercentOutput, 0.0);
				rightConvMotor.Set(ControlMode::PercentOutput, 0.0);
				leftConvMotor.Set(ControlMode::PercentOutput, 0.0);
				startTime = time(NULL);
			}
		}

		if (mode == "deposit"){
			if (intakeArmMotor.GetSensorCollection().IsFwdLimitSwitchClosed() == 0){
				startTime = time(NULL);
			}
			if (intakeArmMotor.GetSensorCollection().IsRevLimitSwitchClosed() == 0){
				intakeArmMotor.Set(ControlMode::PercentOutput, 0.0);
				intakeWheelMotor.Set(ControlMode::PercentOutput, 0.4 * side);
				rightConvMotor.Set(ControlMode::PercentOutput, 0.0);
				leftConvMotor.Set(ControlMode::PercentOutput, 0.0);
				startTime = time(NULL);
			} else {
				if (side > 0){
					if (intakeWheelMotor.GetSensorCollection().IsFwdLimitSwitchClosed() == 1){
						if (time(NULL) - startTime > 4){
							intakeWheelMotor.Set(ControlMode::PercentOutput, 0.4 * side);
						} else {
							intakeWheelMotor.Set(ControlMode::PercentOutput, 0.0);
						}
						rightConvMotor.Set(ControlMode::PercentOutput, 0.0);
						leftConvMotor.Set(ControlMode::PercentOutput, 0.0);
						intakeArmMotor.Set(ControlMode::PercentOutput, -0.5);
					} else {
						if (time(NULL) - startTime > 4){
							intakeWheelMotor.Set(ControlMode::PercentOutput, 0.4 * side);
						} else {
							intakeWheelMotor.Set(ControlMode::PercentOutput, -0.4 * side);
						}
						if (intakeArmMotor.GetSensorCollection().IsFwdLimitSwitchClosed() == 0){
							rightConvMotor.Set(ControlMode::PercentOutput, 0.4);
						}
						leftConvMotor.Set(ControlMode::PercentOutput, 0.0);
						intakeArmMotor.Set(ControlMode::PercentOutput, 0.0);
					}
				} else {
					if (intakeWheelMotor.GetSensorCollection().IsRevLimitSwitchClosed() == 1){
						if (time(NULL) - startTime > 4){
							intakeWheelMotor.Set(ControlMode::PercentOutput, 0.4 * side);
						} else {
							intakeWheelMotor.Set(ControlMode::PercentOutput, 0.0);
						}
						leftConvMotor.Set(ControlMode::PercentOutput, 0.0);
						rightConvMotor.Set(ControlMode::PercentOutput, 0.0);
						intakeArmMotor.Set(ControlMode::PercentOutput, -0.5);
					} else {
						if (time(NULL) - startTime > 4){
							intakeWheelMotor.Set(ControlMode::PercentOutput, 0.4 * side);
						} else {
							intakeWheelMotor.Set(ControlMode::PercentOutput, -0.4 * side);
						}
						if (intakeArmMotor.GetSensorCollection().IsFwdLimitSwitchClosed() == 0){
							leftConvMotor.Set(ControlMode::PercentOutput, -0.4);
						}
						rightConvMotor.Set(ControlMode::PercentOutput, 0.0);
						intakeArmMotor.Set(ControlMode::PercentOutput, 0.0);
					}
				}
			}
		}

		if (mode == "down"){
			intakeWheelMotor.Set(ControlMode::PercentOutput, 0.0);
			rightConvMotor.Set(ControlMode::PercentOutput, 0.0);
			leftConvMotor.Set(ControlMode::PercentOutput, 0.0);
			if (intakeArmMotor.GetSensorCollection().IsRevLimitSwitchClosed() == 0){
				intakeArmMotor.Set(ControlMode::PercentOutput, 0.0);
			} else {
				intakeArmMotor.Set(ControlMode::PercentOutput, -0.7);
			}
		}

		if (mode == "mag_out"){
			intakeWheelMotor.Set(ControlMode::PercentOutput, 0.0);
			rightConvMotor.Set(ControlMode::PercentOutput, 0.4);
			leftConvMotor.Set(ControlMode::PercentOutput, -0.4);
			intakeArmMotor.Set(ControlMode::PercentOutput, 0.0);
		}

		if (mode == "mag_in"){
			intakeWheelMotor.Set(ControlMode::PercentOutput, 0.0);
			rightConvMotor.Set(ControlMode::PercentOutput, -0.4);
			leftConvMotor.Set(ControlMode::PercentOutput, 0.4);
			intakeArmMotor.Set(ControlMode::PercentOutput, 0.0);
		}
	}

	void setMode(std::string cmd){

		//check if command is valid and the rover is not docked before setting
		if (!isDocked){
			if (cmd == "mag_in" || cmd == "mag_out" || cmd == "down" || cmd == "disable" || cmd == "reset" || cmd == "intake" || cmd == "store" || cmd == "deposit" || cmd == "switch_left" || cmd == "switch_right"){
				if (cmd == "deposit"){
					startTime = time(NULL);
				}
				if (cmd == "switch_left"){
					side = -1;
				} else if (cmd == "switch_right"){
					side = 1;
				} else {
					mode = cmd;
				}
			}
		}
		// cout << "Set mode to: '";
		// cout << mode;
		// cout << "' ";
	}
};

Intake intake;

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

void velocityDriveCallback(const geometry_msgs::Twist::ConstPtr& msg){

	geometry_msgs::Vector3 linear = msg->linear;
	geometry_msgs::Vector3 angular = msg->linear;

	double V = msg->linear.x;
	double w = msg->angular.z;


    // 4800 ticks/sec -> 0.2 rot/sec -> 0.12 m/sec
	// return (((velocity / wheelCircumference) * 2048.0) / 10.0) * 12.0;
    // 0.1 m/sec -> 0.16666 rot/sec = 1 rad/sec
    // 0.167 * 2048 / 10 * 12 =
    // ticks per 100ms = 480

    // Angular velocity in rot/sec
	double leftVelocity = (V - w * (wheelBaseWidth / 2.0)) / wheelCircumference;
	double rightVelocity = -(V + w * (wheelBaseWidth / 2.0)) / wheelCircumference;


	//cout << "Setting Wheel Velocity: " << leftVelocity << ", " << rightVelocity<<"\n";

	//double leftVelocity = msg->data[0];
	//double rightVelocity = msg->data[1];
	

	double leftTicksPer100MS = getTicksPer100MSFromVelocity(leftVelocity);
	double rightTicksPer100MS = getTicksPer100MSFromVelocity(rightVelocity);

	//Disable forward movement if the touch sensor is activated

	// if (rightConvMotor.GetSensorCollection().IsFwdLimitSwitchClosed() == 0 || rightTicksPer100MS < 0){
	// 	frontRightDrive.Set(ControlMode::Velocity, rightTicksPer100MS);
	// 	backRightDrive.Set(ControlMode::Velocity, rightTicksPer100MS*0.91);
	// } else {
	// 	frontRightDrive.Set(ControlMode::Velocity, 0.0);
	// 	backRightDrive.Set(ControlMode::Velocity, 0.0);
	// }

	// //mutiply back for slightly larger radius
	// if (rightConvMotor.GetSensorCollection().IsRevLimitSwitchClosed() == 0 || leftTicksPer100MS > 0){
	// 	frontLeftDrive.Set(ControlMode::Velocity, leftTicksPer100MS);
	// 	backLeftDrive.Set(ControlMode::Velocity, leftTicksPer100MS*0.91);
	// } else {
	// 	frontLeftDrive.Set(ControlMode::Velocity, 0.0);
	// 	backLeftDrive.Set(ControlMode::Velocity, 0.0);
	// }

	frontRightDrive.Set(ControlMode::Velocity, rightTicksPer100MS);
	backRightDrive.Set(ControlMode::Velocity, rightTicksPer100MS*0.91);

	frontLeftDrive.Set(ControlMode::Velocity, leftTicksPer100MS);
	backLeftDrive.Set(ControlMode::Velocity, leftTicksPer100MS*0.91);
}

void powerDriveCallback(const std_msgs::Float64MultiArray::ConstPtr& powers){
	double left = powers->data[0];
	double right = powers->data[1];

	frontRightDrive.Set(ControlMode::PercentOutput, right);
	backRightDrive.Set(ControlMode::PercentOutput, right);

	frontLeftDrive.Set(ControlMode::PercentOutput, left);
	backLeftDrive.Set(ControlMode::PercentOutput, left);
}

void intakeCommandCallback(const std_msgs::String::ConstPtr& cmd){
	//cout << "Received intake command";
	intake.setMode(cmd->data);
	//cout << cmd->data;
}

void updateDockedState(){
	if (rightConvMotor.GetSensorCollection().IsFwdLimitSwitchClosed() == 1 || rightConvMotor.GetSensorCollection().IsRevLimitSwitchClosed() == 1){
		isDocked = true;
	} else {
		isDocked = false;
	}
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "canbus");
	ros::NodeHandle n;

	n.param("wheel_base_width", wheelBaseWidth, 0.483);
	n.param("wheel_radius", wheelRadius, 0.1905);


	//ros::Subscriber diagnosticSub = n.subscribe("diagnostics", 10, diagnosticCallback);
	ros::Subscriber velocityDriveSub = n.subscribe("cmd_vel", 10, velocityDriveCallback);
	ros::Subscriber intakeCommandSub = n.subscribe("intake/command", 10, intakeCommandCallback);
	//ros::Subscriber powerDriveSub = n.subscribe("PowerDrive", 10, powerDriveCallback);

	publisher.intakeLSPub = n.advertise<std_msgs::Float64MultiArray>("intake/limitswitches", 10);
	ros::Timer intakeLSTimer = n.createTimer(ros::Duration(1.0 / 50.0), std::bind(&Publisher::publishIntakeLimitSwitches, publisher));

	publisher.batteryVoltagePub = n.advertise<std_msgs::Float64>("battery/voltage", 10);
	ros::Timer batteryVoltageTimer = n.createTimer(ros::Duration(1), std::bind(&Publisher::publishVoltage, publisher));

	publisher.encoderVelocityPub = n.advertise<sensor_msgs::JointState>("encoder/velocity", 10);
	ros::Timer encoderVelocityTimer = n.createTimer(ros::Duration(1.0 / 50.0), std::bind(&Publisher::publishEncoderVelocity, publisher));

	publisher.touchSensorPub = n.advertise<std_msgs::Float64MultiArray>("TouchSensors", 10);
	ros::Timer touchSensorTimer = n.createTimer(ros::Duration(1.0 / 50.0), std::bind(&Publisher::publishTouchSensor, publisher));

	initializeDriveMotors();
	initializeIntakeMotors();

	ros::Rate rate(100);
	while(ros::ok()){
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(500);

		updateDockedState();

		intake.update();

		rate.sleep();
		ros::spinOnce();
	}
  	
	return 0;
}
