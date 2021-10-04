#include "dynamixel_ax12_handler.h"

DynamixelAX12Handler::DynamixelAX12Handler(uint8_t device_id, char* device_name, int baudrate) : device_id_(device_id), device_name_(device_name), baudrate_(baudrate) {
	// Initialize PortHandler instance
	portHandler_ = dynamixel::PortHandler::getPortHandler(device_name_);

	// Initialize PacketHandler instance
	packetHandler_ = dynamixel::PacketHandler::getPacketHandler(1.0);
}

// Open device port
bool DynamixelAX12Handler::OpenPort(){
	if (portHandler_->openPort()) return true;
	else return false;
}

// Set baudrate to given value
bool DynamixelAX12Handler::SetBaudrate(){
	if (portHandler_->setBaudRate(baudrate_)) return true;
	else return false;
}

// Initialize servomotor and return error code if not possible
bool DynamixelAX12Handler::InitServo(){
	if (!OpenPort()) throw DynamixelAX12Exception("could not open servomotor " + std::to_string(device_id_) + " port on " + device_name_);
	if (!SetBaudrate()) throw DynamixelAX12Exception("cannot set servomotor baudrate to "+ std::to_string(baudrate_));
	return true;
}

bool DynamixelAX12Handler::UpdateCurrPosition() {
	// Read present position
	return HandleReadComm(ADDR_AX_PRESENT_POSITION, current_position_);
}

bool DynamixelAX12Handler::UpdateIsMoving() {
	// Read present position
	uint8_t is_moving;
	if (HandleReadComm(ADDR_AX_MOVING, is_moving)){
		is_moving_ = (bool) is_moving;
		return true;
	}
	else{
		return false;
	}
}

bool DynamixelAX12Handler::SendPositionCommand(uint16_t goal_position){
	//Enable torque before sending command
	EnableTorque(true);
	// Send goal position
	return HandleWriteComm(ADDR_AX_GOAL_POSITION, goal_position);
}

bool DynamixelAX12Handler::EnableTorque(bool enable){
	// Send goal position
	return HandleWriteComm(ADDR_AX_TORQUE_ENABLE, (uint8_t) enable);
}

bool DynamixelAX12Handler::SetSpeed(uint16_t speed){
	// Set servomotor speed
	return HandleWriteComm(ADDR_AX_MOVING_SPEED, speed);
}

bool DynamixelAX12Handler::SetMinMaxAngle(uint16_t min_angle, uint16_t max_angle){
	//Disable torque to change value
	EnableTorque(false);
	// Send command to set min and max angles
	return (HandleWriteComm(ADDR_AX_CW_LIMIT, min_angle) && HandleWriteComm(ADDR_AX_CCW_LIMIT, max_angle));
}

bool DynamixelAX12Handler::SetCompliance(uint8_t cw_margin, uint8_t ccw_margin, uint8_t cw_slope=32, uint8_t ccw_slope=32, uint16_t punch=32){
	//Disable torque to change value
	EnableTorque(false);
	// Set compliance values: min/max margins, min/max slop and punnch
	return (HandleWriteComm(ADDR_AX_CW_COMPLIANCE_MARGIN, cw_margin)
			&& HandleWriteComm(ADDR_AX_CCW_COMPLIANCE_MARGIN, ccw_margin)
			&& HandleWriteComm(ADDR_AX_CW_COMPLIANCE_SLOPE, cw_slope)
			&& HandleWriteComm(ADDR_AX_CCW_COMPLIANCE_SLOPE, ccw_slope)
			&& HandleWriteComm(ADDR_AX_PUNCH, punch));
}

bool DynamixelAX12Handler::HandleReadComm(uint8_t address, uint8_t& data){
	uint8_t dxl_error;
	int comm_result;

	comm_result = packetHandler_->read1ByteTxRx(portHandler_, device_id_, address, &data, &dxl_error);

	return CheckForError(comm_result, dxl_error);
}

bool DynamixelAX12Handler::HandleReadComm(uint8_t address, uint16_t& data){
	uint8_t dxl_error;
	int comm_result;

	comm_result = packetHandler_->read2ByteTxRx(portHandler_, device_id_, address, &data, &dxl_error);

	return CheckForError(comm_result, dxl_error);
}

bool DynamixelAX12Handler::HandleWriteComm(uint8_t address, const uint8_t& data){
	uint8_t dxl_error;
	int comm_result;

	comm_result = packetHandler_->write1ByteTxRx(portHandler_, device_id_, address, data, &dxl_error);
	
	return CheckForError(comm_result, dxl_error);
}

bool DynamixelAX12Handler::HandleWriteComm(uint8_t address, const uint16_t& data){
	uint8_t dxl_error;
	int comm_result;

	comm_result = packetHandler_->write2ByteTxRx(portHandler_, device_id_, address, data, &dxl_error);
	
	return CheckForError(comm_result, dxl_error);
}

bool DynamixelAX12Handler::CheckForError(int comm_result, uint8_t dxl_error){
	// If comm or istruction failed, thrwo exception with error string
	if (comm_result != COMM_SUCCESS) throw DynamixelAX12Exception(packetHandler_->getTxRxResult(comm_result));
	else if (dxl_error != 0) throw DynamixelAX12Exception(packetHandler_->getRxPacketError(dxl_error));
	else return true;
}


const int DynamixelAX12Handler::CurrPosition() {
	UpdateCurrPosition();
	return current_position_;
}

const bool DynamixelAX12Handler::IsMoving(){
	UpdateIsMoving();
	return is_moving_;
}
