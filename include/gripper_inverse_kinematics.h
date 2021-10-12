#ifndef GRIPPER_INVERSE_KINEMATICS_H
#define GRIPPER_INVERSE_KINEMATICS_H

#include <cmath>
#include <iostream>
#include <map>

using namespace std;

/* Gripper dimensions

Needed dimensions to compute the
forward kinematics of the gripper
*/
struct gripper_dimensions
{
	// In millimeters
	const float a = 53.63;
	const float b = 75.95;
	const float c = 19.0;
	const float u = 11.5;
	const float v = 0.0;
	const float p = 2.25;
	const float q = -29.5;
	const float r = 8.5;
	const float s = 20.0;
	// In rad
	const float psi_offset = 0.7946;
};

/* Forward kinematics solver class

Computes the distance between fingers
given a servo angle
*/
class GripperFKSolver{
public:
	GripperFKSolver(float a, float b, float c, float u, float v, float p, float q, float r, float s, float psi_offset);
	float ComputeDistance(float psi);
private:
	float ComputeAx(float psi);

	// In millimeters
	float a_;
	float b_;
	float c_;
	float u_;
	float v_;
	float p_;
	float q_;
	float r_;
	float s_;
	float g_;
	// In radians
	float psi_offset_;
	float sigma_;
};

/* Inverse kinematics solver class

Builds a lookup table from the
forward kinematics and return the servo
angle given the distance between fingers
*/
class GripperInverseKinematics{
public:
	GripperInverseKinematics(uint16_t min_step, uint16_t max_step);
	uint16_t GetStepsFromPosition(float distance);

private:
	float StepToAngle(uint16_t steps);

	map<float,int> generatedDistances_;		// IK lookup table

	const gripper_dimensions kGripperDim;
	const uint16_t step_offset_;			// Step when gripper is open
	const float step_ratio_ = 0.00511327;	// Ratio to convert from angle (rad) to steps
	const uint8_t step_range_;				// Step range
};

#endif //GRIPPER_INVERSE_KINEMATICS_H