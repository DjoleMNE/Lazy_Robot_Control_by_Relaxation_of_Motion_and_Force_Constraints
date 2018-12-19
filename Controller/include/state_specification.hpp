/*
Author(s): Djordje Vukcevic, Sven Schneider
Institute: Hochschule Bonn-Rhein-Sieg

Copyright (c) [2018]

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef STATE_SPECIFICATION_HPP
#define STATE_SPECIFICATION_HPP
#include <kdl/kdl.hpp>
#include <kdl/kinfam_io.hpp>
#include "kdl/frames.hpp"
#include <kdl/frames_io.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <solver_vereshchagin.hpp>
#include <Eigen/StdVector>

class state_specification
{
	public:
		KDL::JntArray q;
		KDL::JntArray qd;
		KDL::JntArray qdd; 
		KDL::JntArray feedforward_torque;
		KDL::JntArray control_torque;
		KDL::Jacobian ee_unit_constraint_force;
		KDL::JntArray ee_acceleration_energy;
		KDL::Wrenches external_force;
		std::vector<KDL::Frame> frame_pose;
		std::vector<KDL::FrameVel> frame_velocity;
		std::vector<KDL::Twist> frame_acceleration;

		state_specification(const int number_of_joints, 
							const int number_of_segments,
							const int number_of_frames,
							const int number_of_constraints):
			NUMBER_OF_JOINTS_(number_of_joints),
			NUMBER_OF_SEGMENTS_(number_of_segments),
			NUMBER_OF_FRAMES_(number_of_frames),
			NUMBER_OF_CONSTRAINTS_(number_of_constraints)
		{
			q.resize(NUMBER_OF_JOINTS_);
			qd.resize(NUMBER_OF_JOINTS_);
			qdd.resize(NUMBER_OF_JOINTS_);
			feedforward_torque.resize(NUMBER_OF_JOINTS_);
			ee_unit_constraint_force.resize(NUMBER_OF_CONSTRAINTS_); //alpha
			ee_acceleration_energy.resize(NUMBER_OF_CONSTRAINTS_); //beta
			external_force.resize(NUMBER_OF_SEGMENTS_);
			frame_pose.resize(NUMBER_OF_SEGMENTS_);
			frame_velocity.resize(NUMBER_OF_SEGMENTS_);
			frame_acceleration.resize(NUMBER_OF_FRAMES_);
			reset_values();
		};

		state_specification& operator=(const state_specification &rhs)
		{
			if(this != &rhs){
				this->q = rhs.q;
				this->qd = rhs.qd;
				this->qdd = rhs.qdd;
				this->feedforward_torque = rhs.feedforward_torque;
				this->control_torque = rhs.control_torque;
				this->ee_unit_constraint_force = rhs.ee_unit_constraint_force;
				this->ee_acceleration_energy = rhs.ee_acceleration_energy;
				this->external_force = rhs.external_force;
				this->frame_pose = rhs.frame_pose;
				this->frame_velocity = rhs.frame_velocity;
				this->frame_acceleration = rhs.frame_acceleration;
			}
		}

		~state_specification(){}

		void reset_values()
		{
			//External forces on the arm
			for (int i = 0; i < NUMBER_OF_SEGMENTS_; i++)
				KDL::SetToZero(this->external_force[i]);
			
			// Joint space varibles
			for (int i = 0; i < NUMBER_OF_JOINTS_; i++){
				this->q(i) = 0.0;
				this->qd(i) = 0.0;
				this->qdd(i) = 0.0;
				this->feedforward_torque(i) = 0.0;
			}

			//Acceleration Constraints on the End-Effector
			KDL::Twist unit_constraint_force(
				KDL::Vector(0.0, 0.0, 0.0),  // linear
				KDL::Vector(0.0, 0.0, 0.0)); // angular

			for(int i = 0; i < NUMBER_OF_CONSTRAINTS_; i++)
			{
				this->ee_unit_constraint_force.setColumn(i, 
														 unit_constraint_force);
				this->ee_acceleration_energy(i) = 0.0;
			}

			// Accelerations, velocities and poses of segments
			for (int i = 0; i < NUMBER_OF_FRAMES_; i++)
				KDL::SetToZero(this->frame_acceleration[i]);
		}

		private:
			const int NUMBER_OF_JOINTS_;
			const int NUMBER_OF_SEGMENTS_;
			const int NUMBER_OF_FRAMES_;
			const int NUMBER_OF_CONSTRAINTS_;
};
#endif /* STATE_SPECIFICATION_HPP */