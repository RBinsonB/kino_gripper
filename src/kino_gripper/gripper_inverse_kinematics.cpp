#include <cmath>
#include <iostream>
#include <map>

using namespace std;

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

float Deg2Rad(float deg)
{
    return deg * M_PI / 180.0;
}

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

GripperFKSolver::GripperFKSolver(float a, float b, float c, float u, float v, float p, float q, float r, float s, float psi_offset): a_(a),
 								 b_(b), c_(c), u_(u), v_(v), p_(p), q_(q), r_(r), s_(s), psi_offset_(psi_offset) {
	g_ = sqrt(pow(p,2) + pow(q,2));
	sigma_ = acos(p/g_);
}

float GripperFKSolver::ComputeAx(float psi){
	float f = sqrt(-2*g_*b_*-cos(sigma_+psi) + pow(g_,2) + pow(b_,2));
	float gamma = acos((pow(g_,2) + pow(f,2) - pow(b_,2)) / (2*g_*f)) - sigma_;
	float e = (pow(f,2) + pow(c_,2) - pow(a_,2)) / (2*f);
	float Dy = v_ + (f-e)*sin(gamma);
	float h = sqrt(pow(c_,2) - pow(e,2));
	float Ay = Dy + h*cos(-gamma);
	float Bx = u_ + p_ + b_*cos(psi);
	float By = v_ + q_ + b_*sin(psi);
	float z = pow(Ay,2) - 2*Ay*By - pow(u_,2) + 2*u_*Bx + pow(v_,2) + 2*v_*By - pow(c_,2) + pow(f,2);
	return Bx - sqrt(pow(Bx,2)-z);
}

float GripperFKSolver::ComputeDistance(float psi){
	return (ComputeAx(psi+psi_offset_) - r_)*2;
}

class GripperInverseKinematics{
public:
	GripperInverseKinematics();
	uint16_t GetStepsFromPosition(float distance);

private:
	float StepToAngle(uint16_t steps);

	map<float,int> generatedDistances_;		// IK lookup table

	const gripper_dimensions kGripperDim;
	const uint16_t step_offset_ = 512;		// Step when gripper is open
	const float step_ratio_ = 0.00511327;	// Ratio to convert from angle (rad) to steps
	const uint8_t step_range_ = 120;		// Step range
};

GripperInverseKinematics::GripperInverseKinematics(){
	// Forward kinematics are used to create a lookup table for inverse kinematics
	GripperFKSolver fk_solver_(kGripperDim.a, kGripperDim.b, kGripperDim.c, kGripperDim.u, kGripperDim.v, kGripperDim.p, kGripperDim.q, kGripperDim.r, kGripperDim.s, kGripperDim.psi_offset);

	// Generate distance for every angle steps and populate lookup table
	for (int i=0; i<=step_range_; ++i){
		generatedDistances_[fk_solver_.ComputeDistance(StepToAngle(i))] = step_offset_ - i;	// steps start at offset and decrease to increase angle and close gripper
		cout << "Adding distance " << fk_solver_.ComputeDistance(StepToAngle(i)) << "mm for step " << step_offset_ - i << ", angle of " << StepToAngle(i) << "rad" << endl;
	}
}

// Convert servomotor step to angle
// For Dynamixel AX-12
float GripperInverseKinematics::StepToAngle(uint16_t steps){
	return (step_ratio_ * steps);
}

// Returns the step cmd to close the gripper to a specific distance between fingers
uint16_t GripperInverseKinematics::GetStepsFromPosition(float distance){
	auto it = generatedDistances_.upper_bound(distance);		// Find closest distance (smaller) in lookup table
	return (*it).second;
}