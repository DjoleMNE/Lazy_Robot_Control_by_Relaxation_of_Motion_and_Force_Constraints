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

#ifndef LWR_MEDIATOR_HPP
#define LWR_MEDIATOR_HPP
#include <robot_mediator.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <lwr_kdl_model.hpp>
#include <constants.hpp>

enum lwr_model 
{
    LWR_URDF = 0,
	LWR_WITH_ATI = 1,
    LWR_KDL = 2  
};

enum lwr_environment 
{
    LWR_REAL,
    LWR_SIMULATION   
};

class lwr_mediator: public robot_mediator
{
	public:
		lwr_mediator();
		~lwr_mediator(){}

		// Initializes variables and calibrates the manipulator
		virtual void initialize(const int robot_model,
								const int robot_environment,
                                const bool gravity_compensated);
		
		virtual bool is_initialized();
		virtual int get_robot_ID();

		// Set desired joint commands to move robot and save them for sake of simulation
		virtual void set_joint_command(const KDL::JntArray &joint_positions,
							   const KDL::JntArray &joint_velocities,
							   const KDL::JntArray &joint_torques,
							   const int desired_control_mode);

		// Get current joint positions
		virtual void get_joint_positions(KDL::JntArray &joint_positions);
		// Get current joint velocities 
		virtual void get_joint_velocities(KDL::JntArray &joint_velocities);
		// Get current joint torques
		virtual void get_joint_torques(KDL::JntArray &joint_torques);

		// Set joint position command
		virtual void set_joint_positions(const KDL::JntArray &joint_positions);
		// Set joint velocity command
		virtual void set_joint_velocities(const KDL::JntArray &joint_velocities);
		// Set joint torque command
		virtual void set_joint_torques(const KDL::JntArray &joint_torques); 

		virtual std::vector<double> get_maximum_joint_pos_limits();
		virtual std::vector<double> get_minimum_joint_pos_limits();
		virtual std::vector<double> get_joint_position_thresholds();
		virtual std::vector<double> get_joint_velocity_limits();
		virtual std::vector<double> get_joint_torque_limits();
		virtual std::vector<double> get_joint_inertia();
		virtual std::vector<double> get_joint_offsets();
		
		virtual KDL::Twist get_root_acceleration();
		virtual KDL::Chain get_robot_model();

	private:
		const int ROBOT_ID_;
		bool add_offsets_;
		bool is_initialized_;
		bool connection_established_;
		int lwr_model_;
		int lwr_environment_;

		KDL::Chain lwr_chain_;		
		KDL::Tree lwr_tree_;
    	urdf::Model lwr_urdf_model_;

		//Arm's root acceleration
		const KDL::Vector linear_root_acc_;
		const KDL::Vector angular_root_acc_;
		KDL::Twist root_acc_;

		// Joint Measured State Variables
        Eigen::VectorXd q_measured_;
        Eigen::VectorXd qd_measured_;
        Eigen::VectorXd tau_measured_;
        
        // Joint Setpoint Variables
        Eigen::VectorXd q_setpoint_;
        Eigen::VectorXd qd_setpoint_;
        Eigen::VectorXd tau_setpoint_;

        //Extract LWR model from urdf file
        int get_model_from_urdf();
        int get_model_with_ati_from_urdf();
		//Extract LWR model from KDL parameters
		void get_kdl_model();
};
#endif /* LWR_MEDIATOR_HPP */
