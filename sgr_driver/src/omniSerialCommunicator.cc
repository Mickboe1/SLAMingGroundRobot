#include <string>
#include <iostream>
#include <stdlib.h>
#include <cstdio>
#include <unistd.h>
#include "serial/serial.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Joy.h"
#include <sstream>
#include <math.h>
using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

ros::Subscriber subCmd_Vel;
ros::Subscriber subPID;
ros::Subscriber subTimer;
ros::Publisher 	pubImu;
ros::Publisher 	pubSonar;
ros::Publisher 	pubEncoder;
ros::Publisher 	pubInfrared;
ros::Publisher 	pubCalcedsp;


int baudrate = 115200;

string portname = "/dev/ttyUSB0";
string prefix = "/sgr/";
serial::Serial*	serialOmni;
const int cmd_vel_int = 80;
const int pid_int = 81;

string 	inputString;
int  	dataType;
string 	dataString;

vector<double> tosendCmdVel;


geometry_msgs::Twist velocity;
sensor_msgs::Imu imu;
std_msgs::Float32MultiArray wheels;
std_msgs::Float32MultiArray lwheels;

using namespace std;

void publishData(int sensorNumber, vector<double> data){
	try{
		switch(sensorNumber){
			case 0:
				// pubGyro.publish();
//				imu.angular_velocity.x = data.at(0);
//				imu.angular_velocity.y = data.at(1);
//				imu.angular_velocity.z = data.at(2);
				break;
			case 1:
				imu.linear_acceleration.x = data.at(0);
				imu.linear_acceleration.y = data.at(1);
				imu.linear_acceleration.z = data.at(2);
				break;
			case 2:
				imu.orientation.x = data.at(0);
				imu.orientation.y = data.at(1);
				imu.orientation.z = data.at(2);
				pubImu.publish(imu);
				break;
			case 3:
				//pubSonar.publish();
				break;
			case 4:
				if(data.size() == 4 && wheels.data.size() == 4){ //OMNI 4 WHEELER && theres wheel data to compare with.
					int maxOffset = 50;

					for(int i = 0; i < 4; i++){
//						cout << i << " " << data.at(i) << " " << wheels.data.at(i) << '\n';
						if(data.at(i) - wheels.data.at(i) > maxOffset || data.at(i) - wheels.data.at(i) < -maxOffset){
							return;
						}
//						cout << "still here " << i << '\n';
					}
				}
				else if(data.size() == 3 && wheels.data.size() == 3) {
					 // TODO For the omni three wheeler there has to be a check. however in the case
					 // of a four wheeler and the size beeing 3 big were creating a problem. This means that somehow
					 // the amount of wheels have to be detected and this has to be checked as well.
					 // Looking at you Marijn.
					return;
				}
				else if(data.size() <= 3){
					return;
				}
				else{
					// std::cout << "wheels meet criteria staring pushback" << '\n';

				}
				wheels.data.clear();
				if(data.size() == 4){
					for(int i = 0; i < 4; i++){
//						cout << i << " print still here";
						wheels.data.push_back(data.at(i));
					}
				}
				pubEncoder.publish(wheels);


				break;
			case 5:
				//pubInfrared.publish();
				break;
			case 70:
				if(data.size() > 3){
					velocity.linear.x = data.at(0);
					velocity.linear.y = data.at(1);
					velocity.linear.z = data.at(2);
					pubCalcedsp.publish(velocity);
				}
				break;
			default:
				cerr << "INVALID SENSORTYPE: " << sensorNumber << endl;
				break;
		}
	}
	catch (exception &e) {
			cerr << "Unhandled Exception inside publishData: " << sensorNumber << " " << e.what() << endl;
	}

}

vector<double> readData(string input) {
		// std::cout << input << '\n';
    vector<double> data;
    string buffer = "";
    for (int i = 0; i < input.length(); i++) {
        if (input.at(i) == '*') {
            data.push_back(atof(buffer.c_str()));
            buffer = "";
        } else {
            buffer += input.at(i);
        }
    }
    data.push_back(atof(buffer.c_str()));
    return data;
}

void sendData(int dataType, vector<double> data) {
		// std::cout << "got data " << dataType << '\n';
    stringstream toSend;
    toSend << "&" << dataType << ";";
    for (int i = 0; i < data.size(); i++) {
        toSend << data[i];
        if (i < data.size() - 1) {
            toSend << "*";
        }
    }
    toSend << "#" << endl;
	// cout << toSend.str() << endl;
	serialOmni->write(toSend.str());
}

void cmd_velCallback(const geometry_msgs::Twist& data){
	tosendCmdVel.clear();
	tosendCmdVel.push_back(roundf(data.linear.y * 100 ) / 100);
	tosendCmdVel.push_back(roundf(data.linear.x * 100 ) / 100);
	tosendCmdVel.push_back(-roundf(data.angular.z * 100 ) / 100);
	//cout << "callback " << data.linear.x << " " << data.linear.y << " " << data.angular.z << '\n';



}

void cmdVelTimerCallback(const std_msgs::Empty& msg){
	if(tosendCmdVel.size() == 3){
	        cout << "callback " << tosendCmdVel[0] << " " << tosendCmdVel[1] << " " << tosendCmdVel[2] << '\n';

		sendData(cmd_vel_int, tosendCmdVel);
	}
}

/* I know this is not the right message type, but this works... :P */
void pidCallback(const geometry_msgs::Twist& data){
	vector<double> tosend;
	tosend.push_back(data.linear.y);
	tosend.push_back(data.linear.x);
	tosend.push_back(data.linear.z);
	sendData(pid_int, tosend);
}

void init(int argc, char **argv){
	ros::init(argc, argv, "Omni3wdSerial");
	ros::NodeHandle n("beast");

	// // Create publishers for all of th	e sensors
	pubImu 		= n.advertise<sensor_msgs::Imu>(prefix + "IMU", 10);
	pubSonar	= n.advertise<std_msgs::String>(prefix + "sonar", 10);
	pubEncoder	= n.advertise<std_msgs::Float32MultiArray>(prefix + "encoder", 10);
	pubInfrared	= n.advertise<std_msgs::String>(prefix + "infrared", 10);
	pubCalcedsp	= n.advertise<geometry_msgs::Twist>(prefix + "calcedMotorSpeed", 10);

	subCmd_Vel =  n.subscribe("/cmd_vel", 1000, cmd_velCallback);
	subPID = n.subscribe(prefix + "PID", 1000, pidCallback);
	subTimer = n.subscribe("/timer", 1000, cmdVelTimerCallback);
	ros::Rate loop_rate(10);
}

void cycle(serial::Serial* s){
	inputString = s->readline();
	if(inputString.length() == 0 || inputString.at(0) != '&'){
		return;
	}
	dataType = atoi(inputString.substr(1, inputString.find_first_of(';') - 1).c_str());
	dataString = inputString.substr(inputString.find_first_of(';') + 1,
							   inputString.find_first_of('#') - (inputString.find_first_of(';') + 1));
	publishData(dataType, readData(dataString));
}

int main(int argc, char **argv) {
	try {
		serial::Serial pointer(portname, baudrate, serial::Timeout::simpleTimeout(10));
		serialOmni = &pointer;
		init(argc, argv);

		while(ros::ok()){
			cycle(serialOmni);
			ros::spinOnce();
		}
    }
    catch (exception &e) {
        cerr << "Unhandled Exception: " << e.what() << endl;
    }
	return 0;
}
