// lwr_rtt_control - ISIR Sat Feb 23 14:45:06 2019
// Copyright (c) Djordje Vukcevic, All rights reserved.
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3.0 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this library.

#ifndef __LWR_RTT_CONTROL_HPP__
#define __LWR_RTT_CONTROL_HPP__

// Orocos
#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
#include <rtt/Logger.hpp>
#include <rtt/Component.hpp>
// Eigen
#include <Eigen/Dense>
// RTT-ROS Utilities
#include <rtt_ros_kdl_tools/tools.hpp>
#include <rtt_ros_kdl_tools/chain_utils.hpp>

// Custom Controller code
#include <state_specification.hpp>
#include <solver_vereshchagin.hpp>
#include <fk_vereshchagin.hpp>
#include <geometry_utils.hpp>
#include <model_prediction.hpp>
// #include <dynamics_controller.hpp>
#include <utility> 
#include <abag.hpp>
#include <constants.hpp>
#include <kdl_eigen_conversions.hpp>

class LwrRttControl : public RTT::TaskContext{
    public:
        LwrRttControl(const std::string& name);
        virtual ~LwrRttControl(){};
        void updateHook();
        bool configureHook();
    protected:
        // Generic Model that uses ROS param
        rtt_ros_kdl_tools::ChainUtils arm;
        std::shared_ptr<state_specification> robot_state_;
        std::shared_ptr<KDL::Solver_Vereshchagin> hd_solver_;

        // Input ports
        RTT::InputPort<Eigen::VectorXd>  port_joint_position_in,
                                         port_joint_velocity_in,
                                         port_joint_torque_in;
        // Some input variables
        Eigen::VectorXd jnt_pos_in,
                        jnt_vel_in,
                        jnt_trq_in;
        // Output ports
        RTT::OutputPort<Eigen::VectorXd> port_joint_position_cmd_out,
                                         port_joint_velocity_cmd_out,
                                         port_joint_torque_cmd_out;
        // Some output variables
        Eigen::VectorXd jnt_pos_cmd_out,
                        jnt_vel_cmd_out,
                        jnt_trq_cmd_out;
};

#endif
