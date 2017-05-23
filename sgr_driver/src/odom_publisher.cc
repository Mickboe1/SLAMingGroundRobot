#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/Float32MultiArray.h"

double cwheels[4] = {0,0,0,0}; //current wheels
double lwheels[4] = {0,0,0,0}; //last wheels
double dwheels[4] = {0,0,0,0}; //delta wheels

//encoder ticks
double ticksPerRotation = 760; //cpr
double circumferenceWheel = 314; //in mm
double mmToMeter = 0.001;
double axisLenght = 180; //in mm
// double pow2half  = 1.414213; // 2^(1/2) or sin45 or cos45
double pow2half  = 1; // 2^(1/2) or sin45 or cos45

void encoderCallback(const std_msgs::Float32MultiArray& encoders){
  for(int i = 0; i <= 3; i ++){
    cwheels[i] = -encoders.data.at(i);
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");
  // string prefix = "/sgr/";

  ros::NodeHandle n;
  ros::Publisher  pubOdom     =  n.advertise<nav_msgs::Odometry>("odom", 50);
  ros::Subscriber subPID      =   n.subscribe("sgr/encoder", 1000, encoderCallback);
  tf::TransformBroadcaster odom_broadcaster;

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(20.0);

  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();


    // determine change in wheel encoders
    for(int i = 0; i <= 3; i++){
      dwheels[i] = cwheels[i] - lwheels[i];
      lwheels[i] = cwheels[i];
      std::cout << i << " " << dwheels[i] << '\n';
    }

    double dxTicks = (pow2half * (dwheels[0]/2 - dwheels[2]/2))/2 - (pow2half * (dwheels[1]/2 - dwheels[3]/2))/2;
    double dyTicks = (pow2half * (dwheels[0]/2 - dwheels[2]/2))/2 + (pow2half * (dwheels[1]/2 - dwheels[3]/2))/2;


    double dx =  dxTicks * (circumferenceWheel / ticksPerRotation) * mmToMeter;
    double dy =  dyTicks * (circumferenceWheel / ticksPerRotation) * mmToMeter;
    double dth = (dwheels[0] + dwheels[1] + dwheels[2] + dwheels[3])/(4*axisLenght);


    double dt = (current_time - last_time).toSec();
    x += dx;
    y += dy;
    th += dth;



    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;

    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = dx / dt;
    odom.twist.twist.linear.y = dy / dt;
    odom.twist.twist.angular.z = dth / dt;

    //publish the message
    pubOdom.publish(odom);

    last_time = current_time;
    r.sleep();
  }
}
