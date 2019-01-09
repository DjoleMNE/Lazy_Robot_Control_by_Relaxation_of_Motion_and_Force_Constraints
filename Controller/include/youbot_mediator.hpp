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
#include <youbot_custom_model.hpp>
#include <memory>

class youbot_mediator
{
	public:
		bool is_initialized;				
		/*
        	If interface is used with the solver and the custom model: add offsets
        	Else: set the original values
    	*/ // Custom model's home state is not folded - it is candle
		bool add_offsets;

		youbot_mediator();
		~youbot_mediator(){}

		// Initializes variables and calibrates the manipulator
		void initialize(KDL::Chain &arm_chain,
						const std::string config_path,
						const std::string root_name, 
						const std::string tooltip_name,
						const std::string urdf_path,
						const bool custom_model_used,
						const bool simulation_environment);

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

		std::vector<double> get_positive_joint_pos_limits();
		std::vector<double> get_negative_joint_pos_limits();
		std::vector<double> get_joint_vel_limits();
		std::vector<double> get_joint_acc_limits();
		std::vector<double> get_joint_torque_limits();
		std::vector<double> get_joint_inertia();
		std::vector<double> get_joint_offsets();

	private:
		// Number of joints in the manipulator
		const int NUMBER_OF_JOINTS_;
		bool custom_model_used_;
		int parser_result_ = 0;

        //Absolute path to config files 
        std::string config_path_;

		// Handles for the youbot manipulator and kdl urdf parsel
	    std::shared_ptr<youbot::YouBotManipulator> youbot_arm_;
		KDL::Tree yb_tree;
    	urdf::Model yb_model;

		//Kuka youBot store position limit values: positive and negative
		std::vector<double> joint_position_limits_p_ = {2.9496, 1.5707, 2.5481, 1.7889, 2.9234};
		std::vector<double> joint_position_limits_n_ = {-2.9496, -1.1344, -2.6354, -1.7889, -2.9234};
    
		// Robocup Urdf file parameters for velocity and acceleration limits
		std::vector<double> joint_velocity_limits_ = {1.5707, 0.8, 1.0, 1.5707, 1.5707};
		std::vector<double> joint_acceleration_limits_ = {1.5707, 0.8, 1.0, 1.5707, 1.5707};
		
		// JP's max torques
		// std::vector<double> joint_torque_limits = {12.9012, 12.9012, 8.2700, 4.1748, 1.7550};
		// youBot store's max torques 
		std::vector<double> joint_torque_limits_ = {9.5, 9.5, 6.0, 2.0, 1.0};
		// Benjamin Keiser' max torques (fast version)
		// std::vector<double> joint_torque_limits = {17.0, 17.0, 8.0, 2.7, 1.0}};

		// Offsets required for the custom model: Negative Candle config values -Robocup
		// std::vector<double> youbot_joint_offsets = {-2.1642, -1.13446, 2.54818, -1.78896, -2.9234};
		// Offsets required for the custom model: Negative Candle config values -Sven's
		std::vector<double> youbot_joint_offsets_ = {-2.9496, -1.1344, 2.6354, -1.7890, -2.9234};
		// Offsets required for the custom model: Negative Candle config values: keiser's
		// std::vector<double> youbot_joint_offsets = {-2.9496, -1.1344, 2.5481, -1.7889, -2.9234};

		// Rotor inertia - "d" in the algorithm: Computed from youBot store values
		std::vector<double> youbot_joint_inertia_ = {0.33848, 0.33848, 0.13571, 0.04698, 0.01799};
        
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
        int get_robot_model_from_urdf(KDL::Chain &arm_chain, 
                                std::string root_name, 
                                std::string tooltip_name,
                                std::string urdf_path);
};
#endif /* YOUBOT_MEDIATOR_HPP */
