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
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>

// Custom Controller code
#include <state_specification.hpp>
#include <dynamics_controller.hpp>
#include <utility> 

enum desired_pose 
{
    CANDLE = 0,
    NAVIGATION = 1,
    NAVIGATION_2 = 2,
    FOLDED = 3  
};

class LwrRttControl : public RTT::TaskContext{
    public:
        LwrRttControl(const std::string& name);
        virtual ~LwrRttControl(){};
        void updateHook();
        void stopHook();
        bool configureHook();

    private:
        const int RATE_HZ_;
        const int NUM_OF_SEGMENTS_;
        const int NUM_OF_JOINTS_;
        const int NUM_OF_CONSTRAINTS_;
        int environment_, robot_model_;
        int iteration_count_;

        //General Control Parameters
        bool krc_compensate_gravity_;
        int desired_control_mode_, desired_dynamics_interface_, desired_pose_;
        double damper_amplitude_, damper_slope_;
        std::vector<bool> control_dims_;
        Eigen::VectorXd max_cart_force_, max_cart_acc_; 

        // ABAG Parameters
        Eigen::VectorXd error_alpha_, bias_threshold_, bias_step_, gain_threshold_, gain_step_;
        bool saturate_bias_, saturate_u_;

        // State and Driver
        state_specification robot_state_;
        lwr_mediator robot_driver_;

        //Solvers
        std::shared_ptr<dynamics_controller> controller_;
        std::shared_ptr<KDL::ChainDynParam> gravity_solver_;

        // ROS components 
        std::shared_ptr<ros::NodeHandle> rosnode_;
        tf2_ros::StaticTransformBroadcaster static_broadcaster_;
        geometry_msgs::TransformStamped static_transformStamped_;

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

        void visualize_pose(const std::vector<double> &pose);
};

#endif
