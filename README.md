# Kino Gripper
3D printed Kino gripper repository/ROS package.

Kino, from the greek κίνημα kinema ("movement, motion"), is an adaptative gripper using a [Dynamixel AX-12A servomotor](https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/) as actuator. It leverages the abilies of the smart servomotor: compliance, possibility to set torque, velocity... Communication with the servomotor is handled with an integrated [USB2AX](http://www.xevelabs.com/doku.php?id=product:usb2ax:usb2ax). Power is provided by a DC-DC buck converter.

The gripper is designed to be mounted on a [HEBI Robotics Arm](https://www.hebirobotics.com/robot-arms) but can easily be modified to be mounted on other arms.

[Assembly instructions can be found here](/resources/documentation/kino_gripper_assembly.md).

<img src="/resources/documentation/pictures/render.jpg" align="center" width="400"/>

## Features

### Compliance
Torque and position compliance is provided by the servomotor. The compliance values can be set according to the [Dynamixel AX-12A documentation](https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/#compliance-margin-26-27).

### Adaptive grip
Thanks to the spring on each finger, the gripper can fold to encircle small objects. The spring can be changed to allow for easier/harder finger deformation.

<img src="/resources/documentation/pictures/adaptive.png" align="center" width="700"/>

### Changeable finger tips
Finger tips can be change by simply unscrewing/screwing.

<img src="/resources/documentation/pictures/finger_tip_1.png" align="center" width="400"/>

Three finger tip types are provided, but more can easily be designed:
* Default, with zig-zag pattern
* Thin pads, to be printed and completed by gluing a thin rubber pad
* Thick pads, to be printed and completed by gluing a thick foam pad

<img src="/resources/documentation/pictures/finger_tip_2.png" align="center" width="500"/>

## Installation
The gripper package can be used as a ROS or C++ package.

### Dependencies
- [Robot Operating System (ROS)](http://wiki.ros.org)
- [Dynamixel SDK](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/download/). Either through ROS package or direct install.

### Building
To build the package, clone the current repository in your catkin workspace and build it.
```
cd catkin_ws/src
git clone https://github.com/RBinsonB/kino_gripper.git
```
Build your workspace with either *catkin_make* or *catkin build*
```
cd ...
catkin_make
```

## Usage
### ROS Node
The gripper ROS node can be launched using the following command. Don't forget to set the appropriate port, device ID and baudrate parameters beforehand.
```
rosrun kino_gripper kino_gripper_node
```

#### Subscribed Topics
- `gripper_cmd` ([control_msgs/GripperCommand](http://docs.ros.org/en/melodic/api/control_msgs/html/msg/GripperCommand.html))

    Gripper command message. The position field is commanded distance between the fingers. Torque command is not taken into account and the default servomotor torque will be used.

#### Parameters
- `~port` (string, default: "/dev/ttyACM0")

    Servomotor port. For more information check about USB2AX and AX-12A Dynamixel servomotor.
    
- `~baudrate` (int, default: 1000000)

    Servomotor baudrate in bps. For more information check about USB2AX and AX-12A Dynamixel servomotor.
    
- `~device_id` (uint8_t, default: 1)

    Servomotor ID. For more information check about AX-12A Dynamixel servomotor.
    
### C++
The gripper library can be used independently of ROS. To do so, include the appropriate header in your file.
```cpp
#include <kino_gripper.h>

```

To use the gripper, you can create a gripper object using:
```cpp
KinoGripper(int device_id, char const* device_name, int baudrate)
```
The gripper can then be closed or opened using the Close() and Open() functions.
Example:
```cpp
KinoGripper gripper(1, "/dev/ttyACM0", 1000000);

gripper.Close(40.0);              // Close fingers to 40mm
while (!gripper.IsClosed()){ };   // Wait for gripper to finish
gripper.Close(0.0);               // Close fingers to 0mm
while (!gripper.IsClosed()){ };
gripper.Open();                   // Open gripper completely
while (!gripper.IsClosed()){ };
```

#### API
##### Public Member Functions
```cpp
KinoGripper(int device_id, char const* device_name, int baudrate)
  Gripper object constructor.
  Parameters : 
      device_id Dynamixel servomotor device ID
      device_name Dynamixel servomotor port
      baudrate Dynamixel servomotor baudrate in bps
```
```cpp
void InitGripper()
  Initializes the gripper servomotor and build lookup table for position command
```
```cpp
bool Close(float distance)
  Close the gripper to the given distance (between fingers)
  Parameters : 
      distance Distance between fingers in mm
  Returns :
      True if command sent successfully to servomotor
```
```cpp
bool Open()
  Open the gripper fully by sending maximum angle command (can be set SetMinMaxAngle())
  Returns :
      True if command sent successfully to servomotor
```
```cpp
bool IsClosed()
  Check if the gripper is closed at the value commanded via Close(), uses an inner hardcoded threshold.
  Returns :
      True if gripper is closed to the last commanded value
```
```cpp
bool IsMoving()
  Check if the gripper is moving.
  Returns :
      True if gripper is currently in motion
```
```cpp
bool SetSpeed(uint16_t speed)
  Set servomotor rotation speed
  Parameters : 
      speed Servomotor rotation speed in rev/min
  Returns :
      True if operation is successful
```
```cpp
bool SetMinMaxAngle(uint16_t min_angle, uint16_t max_angle)
  Set servomotor minimum and maximum angles
  Parameters : 
      min_angle Minimum servomotor angle in steps
      max_angle Maximum servomotor angle in steps
  Returns :
      True if operation is successful
```
```cpp
bool SetCompliance(uint8_t cw_margin, uint8_t ccw_margin, uint8_t cw_slope, uint8_t ccw_slope, uint16_t punch)
  Set servomotor compliance parameters. For more information, check Dynamixel AX-12A control table for more details.
  Parameters : 
      cw_margin Clockwise compliance margin in deg 
      ccw_margin Counter-clockwise compliance margin in deg
      cw_slope Clockwise compliance slope
      ccw_slope Counter-clockwise compliance slope
      punch Punch
  Returns :
      True if operation is successful
```
