/*
Author(s): Djordje Vukcevic, Sven Schneider
Institute: Hochschule Bonn-Rhein-Sieg
Description: Mediator component for enabling conversion of data types.
Acknowledgment: This sofware component is based on Jeyaprakash Rajagopal's 
master thesis code.

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

#ifndef YOUBOT_MEDIATOR_HPP
#define YOUBOT_MEDIATOR_HPP
#include <youbot_driver/youbot/YouBotManipulator.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <command_specification.hpp>
#include <memory>

class youbot_mediator
{
	public:
		youbot_mediator(std::string config_path);
		~youbot_mediator(){}

		// Initializes variables and calibrates the manipulator
		int initialize(KDL::Chain &arm_chain, 
                        std::string root_name, 
                        std::string tooltip_name,
                        std::string urdf_path);

		// Get current joint positions
		void get_joint_positions(KDL::JntArray &joint_positions);
		// Get current joint velocities 
		void get_joint_velocities(KDL::JntArray &joint_velocities);
		// Get current joint torques
		void get_joint_torques(KDL::JntArray &joint_torques);

		// Set joint position command
		void set_joint_positions(const KDL::JntArray &joint_positions);
		// Set joint velocity command
		void set_joint_velocities(const KDL::JntArray &joint_velocities);
		// Set joint torque command
		void set_joint_torques(const KDL::JntArray &joint_torques); 

	private:
		// Number of joints in the manipulator
		const int NUMBER_OF_JOINTS_ = 5;

        //Absolute path to config files 
        std::string config_path_;

		// Handles for the youbot manipulator and kdl urdf parsel
	    youbot::YouBotManipulator youbot_arm_;
		KDL::Tree yb_tree;
    	urdf::Model yb_model;
        
        // Joint Measured State Variables
        std::vector<youbot::JointSensedAngle> q_measured_;
        std::vector<youbot::JointSensedVelocity> qd_measured_;
        std::vector<youbot::JointSensedTorque> tau_measured_;
        // std::vector<youbot::JointSensedCurrent> current_measured;
        
        // Joint Setpoint Variables
        std::vector<youbot::JointAngleSetpoint> q_setpoint_;
        std::vector<youbot::JointVelocitySetpoint> qd_setpoint_;
        std::vector<youbot::JointTorqueSetpoint> tau_setpoint_;

        //Extract youBot model from urdf file
        int get_robot_model(KDL::Chain &arm_chain, 
                                std::string root_name, 
                                std::string tooltip_name,
                                std::string urdf_path);
};
#endif /* YOUBOT_MEDIATOR_HPP */
