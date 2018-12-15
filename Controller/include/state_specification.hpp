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
		state_specification(){}
		~state_specification(){}

		KDL::JntArray q;
		KDL::JntArray qd;
		KDL::JntArray qdd; 
		KDL::JntArray feedforward_torque;
		KDL::JntArray control_torque;
		KDL::Jacobian ee_unit_constraint_forces;
		KDL::JntArray ee_acceleration_energy;
		KDL::Wrenches external_force;
		std::vector<KDL::Frame> frame_pose;
		std::vector<KDL::FrameVel> frame_velocity;
		std::vector<KDL::Twist> frame_acceleration;

		void init_state(int number_of_joints,
						int number_of_segments,
						int number_of_frames,
						const int NUMBER_OF_CONSTRAINTS)
		{
			q.resize(number_of_joints);
			qd.resize(number_of_joints);
			qdd.resize(number_of_joints);
			feedforward_torque.resize(number_of_joints);
			ee_unit_constraint_forces.resize(NUMBER_OF_CONSTRAINTS); //alpha
			ee_acceleration_energy.resize(NUMBER_OF_CONSTRAINTS); //beta
			external_force.resize(number_of_segments);
			frame_pose.resize(number_of_frames);
			frame_velocity.resize(number_of_frames);
			frame_acceleration.resize(number_of_frames);
		}
};
#endif /* STATE_SPECIFICATION_HPP */