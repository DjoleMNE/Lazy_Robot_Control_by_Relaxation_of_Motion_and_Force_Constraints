/*
Author(s): Djordje Vukcevic, Sven Schneider
Institute: Hochschule Bonn-Rhein-Sieg

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
#include <rtt_rosparam/rosparam.h>

// Custom Controller code
#include <state_specification.hpp>
#include <dynamics_controller.hpp>
#include <utility> 

enum desired_pose 
{
    CANDLE,
    NAVIGATION,
    FOLDED   
};

class LwrRttControl : public RTT::TaskContext{
    public:
        LwrRttControl(const std::string& name);
        virtual ~LwrRttControl(){};
        void updateHook();
        void stopHook();
        bool configureHook();

    private:
        int environment_;
        int robot_model_;
        bool compensate_gravity_;
        int desired_pose_;
        const int RATE_HZ_;
        const int NUM_OF_SEGMENTS_;
        const int NUM_OF_JOINTS_;
        const int NUM_OF_CONSTRAINTS_;
        std::vector<bool> control_dims_;
        state_specification robot_state_;
        lwr_mediator robot_driver_;
        std::shared_ptr<dynamics_controller> controller_;
        std::shared_ptr<KDL::ChainDynParam> gravity_solver_;

    protected:
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
        KDL::JntArray jnt_gravity_trq_out;
};

#endif
