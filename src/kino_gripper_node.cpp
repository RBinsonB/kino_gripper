#include <memory>
#include <limits>
#include <ros/ros.h>
#include "kino_gripper/dynamixel_ax12_handler.h"


// class KinoGripperNode{
// public:
// 	KinoGripperNode();

// protected:
// 	const uint8_t device_id_ = 1;
// 	char* device_name_ = "/dev/ttyUSB0";
// 	int baudrate_ = 1000000;

// 	std::unique_ptr<DynamixelAX12Handler> gripper_servo_;
// 	//void GetParams();
// 	//void SetupSubAndPub();

// };

// KinoGripperNode::KinoGripperNode(){
// 	// Create servomotor handler
// 	gripper_servo_ = std::unique_ptr<DynamixelAX12Handler>(new DynamixelAX12Handler(device_id_, device_name_, baudrate_));

// 	switch(gripper_servo_->InitServo()){
// 		// Servomotor was successfully initialized
// 		case 0: ROS_INFO_STREAM("Kino gripper: servomotor on \"" << device_name_ 
// 								<< "\" was initialized with baudrate " 
// 								<< baudrate_ << "bps");
// 				break;
// 		// Servomotor port cannot be opened
// 		case 1: ROS_FATAL_STREAM("Kino gripper: servomotor port \"" << device_name_ << "\" cannot be opened!");
// 				break;
// 		// Servomotor baud rate cannot be set
// 		case 2: ROS_FATAL_STREAM("Kino gripper: servomotor baudrate cannot be set to " << baudrate_ << "bps!");
// 				break;
// 	}
// }


class KinoGripper{
public:
	KinoGripper(uint8_t device_id, char* device_name, int baudrate, uint8_t close_ratio);

	bool SetSpeed(uint16_t speed);
	bool SetMinMaxAngle(uint16_t min_angle, uint16_t max_angle);
	bool SetCompliance(uint8_t cw_margin, uint8_t ccw_margin, uint8_t cw_slope, uint8_t ccw_slope, uint16_t punch);
	bool Close();
	bool Open();
	bool IsClosed();
	bool IsMoving();
	
	uint8_t close_ratio_ = 255;		//Gripper closing ratio, from 0 to 255, 0 being minimum angle and 255 maximum

protected:
	std::unique_ptr<DynamixelAX12Handler> gripper_servo_;

	void UpdateClosingAngle();
	bool EnableTorque(bool enable);
	virtual void PrintError(std::string& error_msg);
	virtual void PrintWarning(std::string& warning_msg);

	uint16_t min_angle_ = 570;		// Default value for minimum servomotor angle (closed gripper)
	uint16_t max_angle_ = 685;		// Default value for maximum servomotor angle (open gripper)
	uint16_t angle_threshold_ = 3; 	// Angle threshold to consider the gripper closed
	uint16_t closing_angle_;
};

KinoGripper::KinoGripper(uint8_t device_id, char* device_name, int baudrate, uint8_t close_ratio=255) : close_ratio_(close_ratio){
	// Create servomotor handler
	gripper_servo_ = std::unique_ptr<DynamixelAX12Handler>(new DynamixelAX12Handler(device_id, device_name, baudrate));

	// Compute close angle
	UpdateClosingAngle();

	// Initalize servomotor
	gripper_servo_->InitServo();
}

void KinoGripper::UpdateClosingAngle(){
	closing_angle_  = max_angle_ - ((close_ratio_/std::numeric_limits<uint8_t>::max()) * (max_angle_ - min_angle_));
	return;
}

bool KinoGripper::SetSpeed(uint16_t speed){
	try{
		gripper_servo_->SetSpeed(speed);
	}
	catch (DynamixelAX12Exception& e){
		std::string error_msg = "could not set gripper speed, "+std::string(e.what());
		PrintWarning(error_msg);
		return false;
	}
	return true;
}

bool KinoGripper::EnableTorque(bool enable){
	try{
		gripper_servo_->EnableTorque(enable);
	}
	catch (DynamixelAX12Exception& e){
		std::string error_msg = "could not enable gripper torque, "+std::string(e.what());
		PrintWarning(error_msg);
		return false;
	}
	return true;
}

bool KinoGripper::SetMinMaxAngle(uint16_t min_angle, uint16_t max_angle){
	min_angle_ = min_angle;
	max_angle_ = max_angle;
	UpdateClosingAngle();
	try{
		EnableTorque(false);
		gripper_servo_->SetMinMaxAngle(min_angle, max_angle);
	}
	catch (DynamixelAX12Exception& e){
		std::string error_msg = "could not set min-max angles, "+std::string(e.what());
		PrintWarning(error_msg);
		return false;
	}
	return true;
}

bool KinoGripper::SetCompliance(uint8_t cw_margin, uint8_t ccw_margin, uint8_t cw_slope=32, uint8_t ccw_slope=32, uint16_t punch=32){
	try{
		EnableTorque(false);
		gripper_servo_->SetCompliance(cw_margin, ccw_margin, cw_slope, ccw_slope, punch);
	}
	catch (DynamixelAX12Exception& e){
		std::string error_msg = "could not set gripper compliance values, "+std::string(e.what());
		PrintWarning(error_msg);
		return false;
	}
	return true;
}

bool KinoGripper::Close(){
	try{
		EnableTorque(true);
		ROS_INFO_STREAM("Sending command to " << closing_angle_);
		gripper_servo_->SendPositionCommand(closing_angle_);
	}
	catch (DynamixelAX12Exception& e){
		std::string error_msg = "could not close gripper, "+std::string(e.what());
		PrintWarning(error_msg);
		return false;
	}
	return true;
}

bool KinoGripper::Open(){
	try{
		EnableTorque(true);
		ROS_INFO_STREAM("Sending command to " << max_angle_);
		gripper_servo_->SendPositionCommand(max_angle_);
	}
	catch (DynamixelAX12Exception& e){
		std::string error_msg = "could not close gripper, "+std::string(e.what());
		PrintWarning(error_msg);
		return false;
	}
	return true;
}

bool KinoGripper::IsClosed() {
	try{
		// If gripper is not in movement and angle is close to goal, gripper is considered closed
		if (!IsMoving() && (abs(gripper_servo_->CurrPosition() - closing_angle_) <= angle_threshold_)) return true;
		else return false;
	}
	catch (DynamixelAX12Exception& e){
		std::string error_msg = "could not get gripper info, "+std::string(e.what());
		PrintWarning(error_msg);
		return false;
	}
}

bool KinoGripper::IsMoving() {
	try{
		return gripper_servo_->IsMoving();
	}
	catch (DynamixelAX12Exception& e){
		std::string error_msg = "could not get gripper info, "+std::string(e.what());
		PrintWarning(error_msg);
		return false;
	}
}

void KinoGripper::PrintError(std::string& error_msg){
	std::cerr << error_msg << std::endl;
}

void KinoGripper::PrintWarning(std::string& warning_msg){
	std::cerr << warning_msg << std::endl;
}


int main(){
	KinoGripper gripper(1, "/dev/ttyUSB0", 1000000);
	gripper.SetSpeed(50);

	while(true){
		if (!gripper.IsMoving()){
			if (gripper.IsClosed()){
				ROS_INFO_STREAM("Opening gripper");
				gripper.Open();
			}
			else{
				ROS_INFO_STREAM("Closing gripper");
				gripper.Close();
			}
		}
		ros::Duration(1.5).sleep();
	}

	return 0;
}