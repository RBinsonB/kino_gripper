#ifndef KINO_GRIPPER_H
#define KINO_GRIPPER_H

#include <memory>
#include <limits>
#include <string>
#include <iostream>
#include "dynamixel_ax12_handler.h"
#include "gripper_inverse_kinematics.h"

/* Kino Gripper Class

Uses an interface to a Dynamixel AX12 servomotor
to control the gripper 
*/
class KinoGripper{
public:
	KinoGripper(uint8_t device_id, char* device_name, int baudrate, uint8_t close_ratio);

	bool SetSpeed(uint16_t speed);
	bool SetMinMaxAngle(uint16_t min_angle, uint16_t max_angle);
	bool SetCompliance(uint8_t cw_margin, uint8_t ccw_margin, uint8_t cw_slope, uint8_t ccw_slope, uint16_t punch);
	bool Close(float distance);
	bool Open();
	bool IsClosed();
	bool IsMoving();
	
	uint8_t close_ratio_ = 255;		//Gripper closing ratio, from 0 to 255, 0 being minimum angle and 255 maximum

private:
	std::unique_ptr<DynamixelAX12Handler> gripper_servo_;	// Servo handler pointer
	std::unique_ptr<GripperInverseKinematics> ik_solver_;	// Inverse Kinematics

	bool EnableTorque(bool enable);
	virtual void PrintError(std::string& error_msg);
	virtual void PrintWarning(std::string& warning_msg);

	uint16_t min_angle_ = 392;		// Default value for minimum servomotor angle (closed gripper)
	uint16_t max_angle_ = 512;		// Default value for maximum servomotor angle (open gripper)
	uint16_t angle_threshold_ = 3; 	// Angle threshold to consider the gripper closed
	uint16_t speed_ = 50;
	uint8_t cw_margin_ = 1;
	uint8_t ccw_margin_ = 1;
	uint8_t cw_slope_ = 32;
	uint8_t ccw_slope_ = 32;
	uint16_t punch_ = 32;
	uint16_t closing_angle_;
	bool init_ = false; 			// intialization flag

	bool WriteSpeed();
	bool WriteMinMaxAngle();
	bool WriteCompliance();
};

#endif //KINO_GRIPPER_H