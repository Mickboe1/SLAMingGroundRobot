#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include <unistd.h>
#include <iostream>
#include <sstream>

using namespace std;

unsigned int microseconds = 200000;
bool x = true;
bool y = true;
bool toggle = true;

ros::Publisher 	pubCmd_vel;

geometry_msgs::Twist velocity;


void move(float i){
	velocity.linear.x = 0;
	velocity.linear.y = 0;
	velocity.angular.z = 0;
	int modi = 1;
	if (toggle){
		if(x){
			modi = -1;
		}
		else{
			modi = 1;
		}
		velocity.linear.x = i / 100 * modi;
	}
	else{
		if(y){
			modi = -1;
		}
		else{
			modi = 1;
		}
		velocity.linear.y = i / 100 * modi;
	}
	cout << "Sending " << velocity.linear.x << " " << velocity.linear.y << endl;
	pubCmd_vel.publish(velocity);
}

void cycle(){

	for(float i = 10; i < 50; i += 5){
		move(i);
		usleep(microseconds);
	}
	for(float i = 50; i > 0; i -=5){
		move(i);
		usleep(microseconds);

	}
	if (toggle){
		x = !x;
	}
	else{
		y = !y;
	}
	toggle = !toggle;
}



int main(int argc, char **argv){
	ros::init(argc, argv, "omni_square");
	ros::NodeHandle n;
	pubCmd_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

	try{
		while(ros::ok()){
			cycle();
			ros::spinOnce();
		}
	}catch(exception e){
		cerr << e.what() << endl;
	}
	velocity.linear.x = 0;
	velocity.linear.y = 0;
	velocity.angular.z = 0;
	pubCmd_vel.publish(velocity);
	return 0;
}
