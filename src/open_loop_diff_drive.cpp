#include <open_loop_diff_drive/open_loop_diff_drive.h>

MotorTranslator::MotorTranslator(ros::NodeHandle node)
{
	//Configure the publisher and subscriber for this node
	cmdSub = node.subscribe("cmd_vel", 1, &MotorTranslator::cmdCallback, this);
	leftMotorPub = node.advertise<std_msgs::Int8>("left_motor_speed", 10);
    rightMotorPub = node.advertise<std_msgs::Int8>("right_motor_speed", 10);
}

void MotorTranslator::cmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	/* This is a sort of mixing controller, which combines the linear and rotational velocites to produce
	 * drive signals for each motor. This is kind of a draft, so it may have bugs. The algorithm is based on
	 * the one in the ros diff_drive_controller. We don't need wheel radius and seperation because the 
     * output is PWM frequencies, not wheel-edge-velocities */
	std_msgs::Int8 outgoing_msg;
	outgoing_msg.data = (int)(((msg->linear.x + msg->angular.z)/2) * 127);
	leftMotorPub.publish(outgoing_msg);
	outgoing_msg.data = (int)(((msg->linear.x - msg->angular.z)/2) * 127);
	rightMotorPub.publish(outgoing_msg);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "differential_motor_translator");
	ros::NodeHandle node("~");

	MotorTranslator mt = MotorTranslator(node);

	//This node publishes every time it gets a message, so there's no need to delay
	ros::spin();
}