#include <ros/ros.h>
#include <string>
#include <kino_gripper.h>
#include "control_msgs/GripperCommand.h"


class KinoGripperNode: public KinoGripper {
public:
	KinoGripperNode(char const* device_name);
private:
	void SetSubAndPub(ros::NodeHandle& private_nh);
	void GetParam(ros::NodeHandle& private_nh);
	void GripperCmdCallback(const control_msgs::GripperCommand& msg);

	virtual void PrintError(std::string& error_msg);
	virtual void PrintWarning(std::string& warning_msg);

	ros::Subscriber gripper_cmd_sub;

	const uint8_t default_device_id_ = 1;
	const int default_baudrate_ = 1000000;
};

KinoGripperNode::KinoGripperNode(char const* device_name = "/dev/ttyACM0") : KinoGripper(1, device_name, 1000000){
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	GetParam(private_nh);
	InitGripper();				// To execute before setting up command subscriber
	SetSpeed(50);				// Set low speed
	SetSubAndPub(private_nh);
}

void KinoGripperNode::GetParam(ros::NodeHandle& private_nh){
	// Get baudrate
	if(!private_nh.getParam("baudrate", baudrate_)){
		baudrate_ = default_baudrate_;
		ROS_INFO_STREAM("Kino gripper: using default baudrate " << baudrate_ << "bps");
	}

	// Get Dynamixel servomotor device id
	if(!private_nh.getParam("device_id", device_id_)){
		device_id_ = default_device_id_;
		ROS_INFO_STREAM("Kino gripper: using default device id " << device_id_);
	}
}

void KinoGripperNode::SetSubAndPub(ros::NodeHandle& private_nh){
	gripper_cmd_sub = private_nh.subscribe("gripper_cmd", 1000, &KinoGripperNode::GripperCmdCallback, this);
}

void KinoGripperNode::GripperCmdCallback(const control_msgs::GripperCommand& msg){
	Close((msg.position)*1000); // Convert from message (m) to mm
	return;
}

void KinoGripperNode::PrintError(std::string& error_msg){
	ROS_ERROR_STREAM("Kino gripper node: " << error_msg);
}

void KinoGripperNode::PrintWarning(std::string& warning_msg){
	ROS_WARN_STREAM("Kino gripper node: " << warning_msg);
}


int main(int argc, char **argv){
	ros::init(argc, argv, "kino_gripper");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");


	std::string servo_port = "/dev/ttyACM0";	// Default servomotor port
	private_nh.getParam("port", servo_port);	// Try to get port param, use default if unsuccessful
	KinoGripperNode gripper_node(servo_port.c_str());	// Create gripper object

	while(ros::ok()){
		ros::spin();
	}

	ROS_INFO_STREAM("Kino gripper node: shutting down");

	return 0;
}