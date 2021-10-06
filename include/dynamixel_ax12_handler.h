#ifndef DYNAMIXEL_AX12_HANDLER_H
#define DYNAMIXEL_AX12_HANDLER_H

#include <stdexcept>
#include <iostream>
#include <string>
#include "dynamixel_sdk/dynamixel_sdk.h"

/*Dynamixel AX-12 exception class

Return errors when communication with
the Dynamixel AX12 servomotor.

Uses Dynamixel protocol version 1.0
*/
class DynamixelAX12Exception : public std::runtime_error
{
public:
	DynamixelAX12Exception(const std::string& what) : std::runtime_error(what){}
};


/*Dynamixel AX-12 servomotor class

Handle low-level commands with the servomotor
Uses Dynamixel protocol version 1.0
*/
class DynamixelAX12Handler{
public:
	DynamixelAX12Handler(uint8_t device_id, char* device_name, int baudrate);
	bool InitServo();
	bool SetSpeed(uint16_t speed);
	bool SetMinMaxAngle(uint16_t min_angle, uint16_t max_angle);
	bool SetCompliance(uint8_t cw_margin, uint8_t ccw_margin, uint8_t cw_slope, uint8_t ccw_slope, uint16_t punch);
	bool EnableTorque(bool enable);
	bool SendPositionCommand(uint16_t goal_position);
	const bool IsConnected();
	const int CurrPosition();
	const bool IsMoving();

	bool init_ = false;					// initialization flag

private:
	bool OpenPort();
	bool SetBaudrate();
	bool UpdateCurrPosition();
	bool UpdateIsMoving();

	bool CheckForError(int comm_result, uint8_t dxl_error);

	bool HandleReadComm(uint8_t address, uint8_t& data);
	bool HandleReadComm(uint8_t address, uint16_t& data);
	bool HandleWriteComm(uint8_t address, const uint8_t& data);
	bool HandleWriteComm(uint8_t address, const uint16_t& data);

	uint16_t current_position_ = 0;				// current servo position
	bool is_moving_ = false;					// true if servo is trying to reach a position
	bool connected_ = false;					// connection flag

	const uint8_t device_id_;
	const char* device_name_;
	const int baudrate_;

	dynamixel::PortHandler *portHandler_ = nullptr;
	dynamixel::PacketHandler *packetHandler_ = nullptr;

	int comm_result_ = COMM_TX_FAIL;             // Communication result

	//AX12 servomotor control table
	const uint16_t ADDR_AX_CW_LIMIT				 = 6;
	const uint16_t ADDR_AX_CCW_LIMIT			 = 8;
	const uint16_t ADDR_AX_MAX_TORQUE			 = 14;
	const uint16_t ADDR_AX_TORQUE_ENABLE		 = 24;
	const uint16_t ADDR_AX_CW_COMPLIANCE_MARGIN  = 26;
	const uint16_t ADDR_AX_CCW_COMPLIANCE_MARGIN = 27;
	const uint16_t ADDR_AX_CW_COMPLIANCE_SLOPE   = 28;
	const uint16_t ADDR_AX_CCW_COMPLIANCE_SLOPE  = 29;
	const uint16_t ADDR_AX_GOAL_POSITION		 = 30;
	const uint16_t ADDR_AX_MOVING_SPEED			 = 32;
	const uint16_t ADDR_AX_PRESENT_POSITION 	 = 36;
	const uint16_t ADDR_AX_MOVING				 = 46;
	const uint16_t ADDR_AX_PUNCH				 = 48;
};

#endif //DYNAMIXEL_AX12_HANDLER_H