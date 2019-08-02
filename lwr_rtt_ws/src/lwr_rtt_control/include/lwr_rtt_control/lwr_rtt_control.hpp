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
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <kdl_conversions/kdl_msg.h>
#include <visualization_msgs/Marker.h>

// Custom Controller code
#include <state_specification.hpp>
#include <dynamics_controller.hpp>
#include <utility> 

enum desired_pose 
{
    CANDLE       = 0,
    NAVIGATION   = 1,
    NAVIGATION_2 = 2,
    FOLDED       = 3,
    TABLE        = 4,
    CANDLE2      = 5
};

enum path_types
{
    SINE_PATH = 0,
    STEP_PATH = 1,
    INF_SIGN_PATH = 2
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
        int environment_, robot_model_, iteration_count_, simulation_loop_iterations_;
        int gazebo_arm_eef_;

        std::ofstream log_file_ext_force_;

        //Timer
        double total_time_, task_time_limit_sec_;
        std::chrono::steady_clock::time_point start_time_;
        std::chrono::steady_clock::time_point end_time_;

        //General Control Parameters
        bool krc_compensate_gravity_, use_mixed_driver_, load_ati_sensor_;
        int desired_task_model_, desired_control_mode_, desired_dynamics_interface_;
        int desired_pose_, motion_profile_, path_type_;
        double damper_amplitude_, tube_speed_, tube_force_;
        std::vector<bool> control_dims_;
        std::vector< std::vector<double> > tube_path_points_, path_poses_;
        std::vector<double> path_parameters_, desired_ee_pose_, tube_tolerances_, tube_start_position_;
        Eigen::VectorXd max_command_; 
        KDL::Frame gazebo_ee_frame_;
        KDL::Wrench ext_wrench_kdl_;

        // ABAG Parameters
        Eigen::VectorXd error_alpha_, bias_threshold_, bias_step_, 
                        gain_threshold_, gain_step_, min_bias_sat_, min_command_sat_,
                        null_space_abag_parameters_;
        int abag_error_type_;

        // State and Driver
        state_specification robot_state_;
        lwr_mediator robot_driver_;

        //Solvers
        std::shared_ptr<dynamics_controller> controller_;
        std::shared_ptr<KDL::ChainDynParam> gravity_solver_;
        std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;

        rtt_ros_kdl_tools::ChainUtils gazebo_arm_;

        // ROS components 
        std::shared_ptr<ros::NodeHandle> rosnode_;
        tf2_ros::StaticTransformBroadcaster static_broadcaster_;
        geometry_msgs::TransformStamped static_transformStamped_;

    protected:
        // Input ports
        RTT::FlowStatus return_msg_;
        RTT::InputPort<Eigen::VectorXd>  port_joint_position_in,
                                         port_joint_velocity_in,
                                         port_joint_torque_in;

        // Port for reading data from Force-Torque sensor on the end-effector
        RTT::InputPort<geometry_msgs::WrenchStamped> port_ext_force_in;
        
        // Some input variables
        Eigen::VectorXd jnt_pos_in,
                        jnt_vel_in,
                        jnt_trq_in;

        // data from Force-Torque sensor on the end-effector        
        geometry_msgs::WrenchStamped wrench_msg_;
        
        // Output ports
        RTT::OutputPort<Eigen::VectorXd> port_joint_position_cmd_out,
                                         port_joint_velocity_cmd_out,
                                         port_joint_torque_cmd_out;
        // Some output variables
        Eigen::VectorXd jnt_pos_cmd_out,
                        jnt_vel_cmd_out,
                        jnt_trq_cmd_out;
        KDL::JntArray jnt_gravity_trq_out;

        void visualize_pose(const std::vector<double> &pose, 
                            const std::vector<std::vector<double>> &path_poses);
        
        // Path generators for X-Z plane
        void draw_sine_xz(std::vector< std::vector<double> > &path_points,
                          const double frequency_start,
                          const double frequency_end,
                          const double amplitude,
                          const double x_scale, const double offset_x, 
                          const double offset_y, const double offset_z);
        void draw_inf_sign_xz(std::vector< std::vector<double> > &path_points,
                              const double length, const double height, 
                              const double amplitude,
                              const double x_scale, const double offset_x, 
                              const double offset_y, const double offset_z);
        void draw_step_xz(std::vector< std::vector<double> > &path_points,
                          const int step_size,
                          const double x_scale, const double offset_x, 
                          const double offset_y, const double offset_z);

        // Path generators for X-Y plane
        void draw_sine_xy(std::vector< std::vector<double> > &path_points,
                          const double frequency_start,
                          const double frequency_end,
                          const double amplitude,
                          const double x_scale, const double offset_x, 
                          const double offset_y, const double offset_z);
        void draw_inf_sign_xy(std::vector< std::vector<double> > &path_points,
                              const double length, const double height, 
                              const double amplitude,
                              const double x_scale, const double offset_x, 
                              const double offset_y, const double offset_z);
        void draw_step_xy(std::vector< std::vector<double> > &path_points,
                          const int step_size,
                          const double x_scale, const double offset_x, 
                          const double offset_y, const double offset_z);
};

#endif
