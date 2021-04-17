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
#include <state_specification.hpp>
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
#include <model_prediction.hpp>
#include <fd_solver_rne.hpp>

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
								const int id,
                                const double DT_SEC);

		virtual bool is_initialized();

		// De-initializes variables and closes the sessions 
		void deinitialize();

		// Update joint space state: measured positions, velocities and torques
		virtual void get_joint_state(KDL::JntArray &joint_positions,
									 KDL::JntArray &joint_velocities,
									 KDL::JntArray &joint_torques);

		// Update robot state: measured positions, velocities, torques and measured / estimated external forces on end-effector
		virtual void get_robot_state(KDL::JntArray &joint_positions,
                                     KDL::JntArray &joint_velocities,
                                     KDL::JntArray &joint_torques,
                                     KDL::Wrench &end_effector_wrench);

		// Set desired joint commands to move robot and save them for sake of simulation
		virtual int set_joint_command(const KDL::JntArray &joint_positions,
						              const KDL::JntArray &joint_velocities,
							          const KDL::JntArray &joint_torques,
							          const int desired_control_mode);

		// Set joint position command
		virtual int set_joint_positions(const KDL::JntArray &joint_positions);
		// Set joint velocity command
		virtual int set_joint_velocities(const KDL::JntArray &joint_velocities);
		// Set joint torque command
		virtual int set_joint_torques(const KDL::JntArray &joint_torques); 
		// Set Zero Joint Velocities and wait until robot has stopped completely
		virtual int stop_robot_motion();
		// Set desired control mode for robot actuators (position/velocity/torque)
		int set_control_mode(const int desired_control_mode);
		// Set external wrenches for the simulation
		void set_ext_wrenches_sim(const KDL::Wrenches &ext_wrenches_sim);

		virtual std::vector<double> get_maximum_joint_pos_limits();
		virtual std::vector<double> get_minimum_joint_pos_limits();
		virtual std::vector<double> get_joint_position_thresholds();
		virtual std::vector<double> get_joint_velocity_limits();
		virtual std::vector<double> get_joint_acceleration_limits();
		virtual std::vector<double> get_joint_torque_limits();
		virtual std::vector<double> get_joint_stopping_torque_limits();
		virtual std::vector<double> get_joint_inertia();
		virtual std::vector<double> get_joint_offsets();
		virtual int get_robot_ID();
		virtual int get_robot_environment();

		virtual KDL::Twist get_root_acceleration();
		virtual KDL::Chain get_robot_model();
		virtual KDL::Chain get_full_robot_model();

	private:
		bool is_initialized_;
		int kinova_id;
		int kinova_model_;
		int kinova_environment_;
		int control_mode_;
		bool add_offsets_;
		bool connection_established_;
        double DT_SEC_;

		KDL::Chain kinova_chain_, kinova_sim_chain_;		
		KDL::Tree kinova_tree_;
    	urdf::Model kinova_urdf_model_;

        KDL::Wrenches ext_wrenches_sim_;
		state_specification robot_state_;
    	std::vector<state_specification> predicted_states_; 
		std::shared_ptr<model_prediction> predictor_;
		std::shared_ptr<KDL::FdSolver_RNE> fd_solver_rne_;

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

		// Get current joint positions
		virtual void get_joint_positions(KDL::JntArray &joint_positions);
		// Get current joint velocities 
		virtual void get_joint_velocities(KDL::JntArray &joint_velocities);
		// Get current joint torques
		virtual void get_joint_torques(KDL::JntArray &joint_torques);
		// Get measured / estimated external forces acting on the end-effector
		virtual void get_end_effector_wrench(KDL::Wrench &end_effector_wrench);

		// Increses index of the command's frame id (buffer)
		void increment_command_id();

		//Extract kinova model from urdf file
        int get_model_from_urdf();
        int get_sim_model_from_urdf();

		// void error_callback_(Kinova::Api::KError err);

		bool get_bit(unsigned int flag, const int position);
		bool robot_stopped();
};
#endif /* KINOVA_MEDIATOR_HPP */
