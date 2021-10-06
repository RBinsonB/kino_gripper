#include "kino_gripper.h"

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
	speed_ = speed;
	return WriteSpeed();
}

bool KinoGripper::WriteSpeed(){
	try{
		gripper_servo_->SetSpeed(speed_);
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
	return WriteMinMaxAngle();
}

bool KinoGripper::WriteMinMaxAngle(){
	try{
		EnableTorque(false);
		gripper_servo_->SetMinMaxAngle(min_angle_, max_angle_);
	}
	catch (DynamixelAX12Exception& e){
		std::string error_msg = "could not set min-max angles, "+std::string(e.what());
		PrintWarning(error_msg);
		return false;
	}
	return true;	
}

bool KinoGripper::SetCompliance(uint8_t cw_margin, uint8_t ccw_margin, uint8_t cw_slope=32, uint8_t ccw_slope=32, uint16_t punch=32){
	cw_margin_ = cw_margin;
	ccw_margin_ = ccw_margin;
	cw_slope_ = cw_slope;
	ccw_slope_ = ccw_slope;
	punch_ = punch;
	return WriteCompliance();
}

bool KinoGripper::WriteCompliance(){
	try{
		EnableTorque(false);
		gripper_servo_->SetCompliance(cw_margin_, ccw_margin_, cw_slope_, ccw_slope_, punch_);
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
		WriteSpeed();
		WriteCompliance();
		EnableTorque(true);
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
		WriteSpeed();
		WriteCompliance();
		EnableTorque(true);
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