#include "gripper_inverse_kinematics.h"

/* Forward kinematics solver class

Computes the distance between fingers
given a servo angle
*/

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


/* Inverse kinematics solver class

Builds a lookup table from the
forward kinematics and return the servo
angle given the distance between fingers
*/

GripperInverseKinematics::GripperInverseKinematics(uint16_t min_step, uint16_t max_step) : step_offset_(max_step), step_range_(max_step-min_step) {
	// Forward kinematics are used to create a lookup table for inverse kinematics
	GripperFKSolver fk_solver_(kGripperDim.a, kGripperDim.b, kGripperDim.c, kGripperDim.u, kGripperDim.v, kGripperDim.p, kGripperDim.q, kGripperDim.r, kGripperDim.s, kGripperDim.psi_offset);

	// Generate distance for every angle steps and populate lookup table
	for (int i=0; i<=step_range_; ++i){
		generatedDistances_[fk_solver_.ComputeDistance(StepToAngle(i))] = step_offset_ - i;	// steps start at offset and decrease to increase angle and close gripper
		//cout << "Adding distance " << fk_solver_.ComputeDistance(StepToAngle(i)) << "mm for step " << step_offset_ - i << ", angle of " << StepToAngle(i) << "rad" << endl; // for debug
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
	if (it == generatedDistances_.end()){
		it = prev(it);		// if at the end, take last value of the map
	}
	return (*it).second;
}