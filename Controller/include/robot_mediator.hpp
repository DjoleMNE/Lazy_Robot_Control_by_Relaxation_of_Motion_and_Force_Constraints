/*
Author(s): Djordje Vukcevic, Sven Schneider
Institute: Hochschule Bonn-Rhein-Sieg
Description: Mediator component for enabling conversion of data types.

Copyright (c) [2019]

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

#ifndef ROBOT_MEDIATOR_HPP
#define ROBOT_MEDIATOR_HPP
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <constants.hpp>

enum control_mode 
{
    TORQUE = 0,
    VELOCITY = 1,
    POSITION = 2, 
    STOP_MOTION = -1   
};

class robot_mediator
{
	public:
		// virtual ~robot_mediator() = 0;

		virtual bool is_initialized() = 0;
		virtual std::string get_robot_ID() = 0;

		// Initializes variables and calibrates the manipulator
		virtual void initialize(const int robot_model,
								const int robot_environment) = 0;

		// Set desired joint commands to move robot and save them for sake of simulation
		virtual void set_joint_command(const KDL::JntArray &joint_positions,
									   const KDL::JntArray &joint_velocities,
									   const KDL::JntArray &joint_torques,
							   		   const int desired_control_mode) = 0;

		// Get current joint positions
		virtual void get_joint_positions(KDL::JntArray &joint_positions)  = 0 ;
		// Get current joint velocities 
		virtual void get_joint_velocities(KDL::JntArray &joint_velocities) = 0;
		// Get current joint torques
		virtual void get_joint_torques(KDL::JntArray &joint_torques) = 0;

		// Set joint position command
		virtual void set_joint_positions(const KDL::JntArray &joint_positions) = 0;
		// Set joint velocity command
		virtual void set_joint_velocities(const KDL::JntArray &joint_velocities) = 0;
		// Set joint torque command
		virtual void set_joint_torques(const KDL::JntArray &joint_torques) = 0; 

		virtual std::vector<double> get_maximum_joint_pos_limits() = 0;
		virtual std::vector<double> get_minimum_joint_pos_limits() = 0;
		virtual std::vector<double> get_joint_position_thresholds() = 0;
		virtual std::vector<double> get_joint_velocity_limits() = 0;
		virtual std::vector<double> get_joint_torque_limits() = 0;
		virtual std::vector<double> get_joint_inertia() = 0;
		virtual std::vector<double> get_joint_offsets() = 0;
		
		virtual KDL::Twist get_root_acceleration() = 0;
		virtual KDL::Chain get_robot_model() = 0;
};
#endif /* ROBOT_MEDIATOR_HPP */
