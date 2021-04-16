#ifndef OPEN_LOOP_DIFF_DRIVE_H_
#define OPEN_LOOP_DIFF_DRIVE_H_

#include <ros/ros.h>
#include <cmath>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h>
#include <vector>

class MotorTranslator
{
	protected:
		//Publishes motor speeds based on twist messages
		ros::Publisher leftMotorPub;
        ros::Publisher rightMotorPub;
		//Listens for twist messages to convert to motor drive messages
		ros::Subscriber cmdSub;
	public:
		//Receives twists and converts them to motor drive signals
		void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg);
		MotorTranslator(ros::NodeHandle node);
};
#endif