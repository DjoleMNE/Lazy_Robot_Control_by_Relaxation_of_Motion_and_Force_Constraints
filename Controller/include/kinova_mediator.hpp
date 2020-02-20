/*
Author(s): Djordje Vukcevic, Sven Schneider
Institute: Hochschule Bonn-Rhein-Sieg
Description: Mediator component for enabling conversion of data types.

Copyright (c) [2020]

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

#ifndef KINOVA_MEDIATOR_HPP
#define KINOVA_MEDIATOR_HPP
#include <robot_mediator.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <constants.hpp>
#include <memory>
#include <iostream>
#include <utility> 
#include <sstream>
#include <fstream>
#include <chrono>
#include <thread> // std::this_thread::sleep_for
#include <time.h>
#include <cmath>
#include <string>
#include <vector>
#include <math.h>
#include <boost/assign/list_of.hpp>
#include <stdlib.h> /* abs */
#include <unistd.h>
#include <Eigen/Dense>// Eigen


#include <KDetailedException.h>

#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <ActuatorConfigClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>

#include <RouterClient.h>
#include <TransportClientUdp.h>
#include <TransportClientTcp.h>

#include <google/protobuf/util/json_util.h>
#define DEG_TO_RAD(x) (x) * PI / 180.0
#define RAD_TO_DEG(x) (x) * 180.0 / PI

enum kinova_model 
{
    URDF = 0,
    CUSTOM = 1
};

enum kinova_environment 
{
    REAL = 0,
    SIMULATION = 1
};

class kinova_mediator: public robot_mediator
{
	public:
		kinova_mediator();
		~kinova_mediator();

		// Initializes variables and calibrates the manipulator
		virtual void initialize(const int robot_model,
								const int robot_environment,
								const bool gravity_compensated);
		virtual void deinitialize();

		virtual bool is_initialized();

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
		// Set Zero Joint Velocities and wait until robot has stopped completely
		virtual void stop_robot_motion();

		virtual std::vector<double> get_maximum_joint_pos_limits();
		virtual std::vector<double> get_minimum_joint_pos_limits();
		virtual std::vector<double> get_joint_position_thresholds();
		virtual std::vector<double> get_joint_velocity_limits();
		virtual std::vector<double> get_joint_torque_limits();
		virtual std::vector<double> get_joint_inertia();
		virtual std::vector<double> get_joint_offsets();
		virtual int get_robot_ID();

		virtual KDL::Twist get_root_acceleration();
		virtual KDL::Chain get_robot_model();

	private:
		bool is_initialized_;
		const int ROBOT_ID_;
		int kinova_model_;
		int kinova_environment_;
		bool add_offsets_;
		bool connection_established_;

		KDL::Chain kinova_chain_;		
		KDL::Tree kinova_tree_;
    	urdf::Model kinova_urdf_model_;

		//Arm's root acceleration
		const KDL::Vector linear_root_acc_;
		const KDL::Vector angular_root_acc_;
		const KDL::Twist root_acc_;
        
		// Handles for the kinova manipulator and kdl urdf parsel
		// Create API objects
	    std::shared_ptr<Kinova::Api::TransportClientTcp> transport_;
	    std::shared_ptr<Kinova::Api::TransportClientUdp> transport_real_time_;
	    std::shared_ptr<Kinova::Api::RouterClient> router_;
	    std::shared_ptr<Kinova::Api::RouterClient> router_real_time_;
	    std::shared_ptr<Kinova::Api::SessionManager> session_manager_;
	    std::shared_ptr<Kinova::Api::SessionManager> session_manager_real_time_;
		std::shared_ptr<Kinova::Api::Base::BaseClient> base_;
	    std::shared_ptr<Kinova::Api::BaseCyclic::BaseCyclicClient> base_cyclic_;
	    std::shared_ptr<Kinova::Api::ActuatorConfig::ActuatorConfigClient> actuator_config_;

        // Joint Measured State Variables
    	Kinova::Api::BaseCyclic::Feedback base_feedback_;
        
        // Joint Setpoint Variables
    	Kinova::Api::BaseCyclic::Command base_command_;

		// Variable for alternating servoing mode (High or Low level control)
		Kinova::Api::Base::ServoingModeInformation servoing_mode_;
		Kinova::Api::ActuatorConfig::ControlModeInformation control_mode_message_;

        //Extract youBot model from urdf file
        int get_model_from_urdf();

		// void error_callback_(Kinova::Api::KError err);

		bool get_bit(unsigned int flag, const int position);
		bool robot_stopped();
};
#endif /* KINOVA_MEDIATOR_HPP */
