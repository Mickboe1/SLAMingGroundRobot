#include <string>
#include <iostream>
#include <stdlib.h>
#include <cstdio>
#include <unistd.h>
#include "serial/serial.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Joy.h"
#include <sstream>

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

ros::Subscriber subCmd_Vel;
ros::Subscriber subPID;
ros::Publisher 	pubImu;
ros::Publisher 	pubSonar;
ros::Publisher 	pubEncoder;
ros::Publisher 	pubInfrared;
ros::Publisher 	pubCalcedsp;


int baudrate = 115200;
string portname = "/dev/ttyUSB0";
serial::Serial*	serialOmni;
const int cmd_vel_int = 80;
const int pid_int = 81;

string 	inputString;
int  	dataType;
string 	dataString;

geometry_msgs::Twist velocity;
sensor_msgs::Imu imu;

using namespace std;

void publishData(int sensorNumber, vector<double> data){
	switch(sensorNumber){
		case 0:
			// pubGyro.publish();
			imu.angular_velocity.x = data.at(0);
			imu.angular_velocity.y = data.at(1);
			imu.angular_velocity.z = data.at(2);
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
			velocity.linear.x = data.at(0);
			velocity.linear.y = data.at(1);
			velocity.linear.z = data.at(2);
			pubEncoder.publish(velocity);
			break;
		case 5:
			//pubInfrared.publish();
			break;
		case 70:
			velocity.linear.x = data.at(0);
			velocity.linear.y = data.at(1);
			velocity.linear.z = data.at(2);
			pubCalcedsp.publish(velocity);
			break;
		default:
			cerr << "INVALID SENSORTYPE: " << sensorNumber << endl;
			break;
	}
}

vector<double> readData(string input) {
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
    stringstream toSend;
    toSend << "&" << dataType << ";";
    for (int i = 0; i < data.size(); i++) {
        toSend << data[i];
        if (i < data.size() - 1) {
            toSend << "*";
        }
    }
    toSend << "#" << endl;
	cout << toSend.str() << endl;
	serialOmni->write(toSend.str());
}

void cmd_velCallback(const geometry_msgs::Twist& data){
	vector<double> tosend;
	tosend.push_back(data.linear.x);
	tosend.push_back(data.linear.y);
	tosend.push_back(data.angular.z);
	// cout << data << endl;
	sendData(cmd_vel_int, tosend);
}


/* I know this is not the right message type, but this works... :P */
void pidCallback(const geometry_msgs::Twist& data){
	vector<double> tosend;
	tosend.push_back(data.linear.x);
	tosend.push_back(data.linear.y);
	tosend.push_back(data.linear.z);
	sendData(pid_int, tosend);
}

void init(int argc, char **argv){
	ros::init(argc, argv, "Omni3wdSerial");
	ros::NodeHandle n("beast");

	// // Create publishers for all of the sensors
	pubImu 		= n.advertise<sensor_msgs::Imu>("/beast/IMU", 10);
	pubSonar	= n.advertise<std_msgs::String>("/beast/sonar", 10);
	pubEncoder	= n.advertise<geometry_msgs::Twist>("/beast/encoder", 10);
	pubInfrared	= n.advertise<std_msgs::String>("/beast/infrared", 10);
	pubCalcedsp	= n.advertise<geometry_msgs::Twist>("/beast/calcedMotorSpeed", 10);

	subCmd_Vel =  n.subscribe("/beast/cmd_vel", 1000, cmd_velCallback);
	subPID = n.subscribe("/beast/PID", 1000, pidCallback);
	ros::Rate loop_rate(1);
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
			ros::spin();
		}
    }
    catch (exception &e) {
        cerr << "Unhandled Exception: " << e.what() << endl;
    }
	return 0;
}
