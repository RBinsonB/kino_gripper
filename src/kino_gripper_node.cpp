#include <ros/ros.h>
#include <kino_gripper.h>


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



int main(int argc, char **argv){
	ros::init(argc, argv, "gripper");
	ros::NodeHandle nh;

	KinoGripper gripper(1, "/dev/ttyACM0", 1000000, 255);
	gripper.SetSpeed(50);

	while(ros::ok()){
		if (!gripper.IsMoving()){
			if (gripper.IsClosed()){
				ROS_INFO_STREAM("Opening gripper");
				gripper.Open();
			}
			else{
				ROS_INFO_STREAM("Closing gripper");
				gripper.Close(0);
			}
		}
		ros::Duration(1.5).sleep();
	}

	return 0;
}