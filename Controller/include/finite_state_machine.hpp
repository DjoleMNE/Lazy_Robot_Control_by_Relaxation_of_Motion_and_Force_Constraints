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

#ifndef FINITE_STATE_MACHINE_HPP
#define FINITE_STATE_MACHINE_HPP
#include <state_specification.hpp>
#include <constants.hpp>
#include <fk_vereshchagin.hpp>
#include <kdl_eigen_conversions.hpp>
#include <motion_profile.hpp>
#include <moving_variance.hpp>
#include <moving_slope.hpp>
#include <iostream>
#include <sstream>
#include <fstream>
#include <unistd.h>
#include <cmath>
#include <stdlib.h>     /* abs */

enum task_model
{
  full_pose = 0,
  moveGuarded = 1,
  moveTo = 2,
  moveTo_follow_path = 3,
  moveConstrained = 4,
  moveConstrained_follow_path = 5,
  moveTo_weight_compensation = 6,
  gravity_compensation = 7
};

enum task_status
{
    STOP_CONTROL        = -1,
    NOMINAL             =  0,
    START_TO_CRUISE     =  1,
    CRUISE_TO_STOP      =  2,
    CRUISE_THROUGH_TUBE =  3,
    CRUISE              =  4,
    CHANGE_TUBE_SECTION =  5,
    APPROACH            =  7,
    STOP_ROBOT          =  6
};


struct moveConstrained_follow_path_task
{
    std::vector<KDL::Frame> tf_poses, goal_poses;
    KDL::Rotation tf_force, null_space_plane_orientation;
    KDL::Vector null_space_force_direction;
    std::vector< std::vector<double> > tube_path_points{1, std::vector<double>(3, 0.0)};
    std::vector<double> tube_tolerances{std::vector<double>(8, 0.0)};
    double null_space_tolerance = 0.0;  // Tolerance unit in degrees
    double tube_speed = 0.0;
    double tube_force = 0.0;
    double contact_threshold_linear = 0.0;
    double contact_threshold_angular = 0.0;
    double time_limit = 0.0;
};


struct moveTo_follow_path_task
{
    std::vector<KDL::Frame> tf_poses, goal_poses;
    std::vector< std::vector<double> > tube_path_points{1, std::vector<double>(3, 0.0)};
    std::vector<double> tube_tolerances{std::vector<double>(7, 0.0)};
    KDL::Vector null_space_force_direction;
    double null_space_tolerance = 0.0;  // Tolerance unit in degrees
    double tube_speed = 0.0;
    double contact_threshold_linear = 0.0;
    double contact_threshold_angular = 0.0;
    double time_limit = 0.0;
};

struct moveTo_task
{
    KDL::Frame tf_pose, goal_pose;
    std::vector<double> tube_start_position{std::vector<double>(3, 0.0)};
    std::vector<double> tube_tolerances{std::vector<double>(7, 0.0)};
    KDL::Vector null_space_force_direction;
    double null_space_tolerance = 0.0;  // Tolerance unit in degrees
    double tube_speed = 0.0;
    double contact_threshold_linear = 0.0;
    double contact_threshold_angular = 0.0;
    double time_limit = 0.0;
};

struct moveGuarded_task
{
    KDL::Frame tf_pose, goal_pose;
    std::vector<double> tube_start_position{std::vector<double>(3, 0.0)};
    std::vector<double> tube_end_position{std::vector<double>(3, 0.0)};
    std::vector<double> tube_tolerances{std::vector<double>(7, 0.0)};
    KDL::Vector null_space_force_direction;
    double null_space_tolerance = 0.0;  // Tolerance unit in degrees
    double tube_speed = 0.0;
    double contact_threshold_linear = 0.0;
    double contact_threshold_angular = 0.0;
    double time_limit = 0.0;
};

struct moveTo_weight_compensation_task
{
    KDL::Frame tf_pose, goal_pose;
    std::vector<double> tube_start_position{std::vector<double>(3, 0.0)};
    std::vector<double> tube_tolerances{std::vector<double>(7, 0.0)};
    KDL::Vector null_space_force_direction;
    double null_space_tolerance = 0.0;  // Tolerance unit in degrees
    double tube_speed = 0.0;
    double contact_threshold_linear = 0.0;
    double contact_threshold_angular = 0.0;
    double time_limit = 0.0;
    bool use_mass_alternation = false;
};

struct full_pose_task
{
    KDL::Frame tf_pose, goal_pose;
    std::vector<double> goal_area{std::vector<double>(6, 0.0)};
    KDL::Vector null_space_force_direction;
    double null_space_tolerance = 0.0;  // Tolerance unit in degrees
    double contact_threshold_linear = 0.0;
    double contact_threshold_angular = 0.0;
    double time_limit = 0.0;
};

struct gravity_compensation_task
{
    double time_limit = 0.0;
};

class finite_state_machine
{
    public:
        finite_state_machine(const int num_of_joints,
                             const int num_of_segments,
                             const int num_of_frames,
                             const int num_of_constraints);
        ~finite_state_machine(){};
        int initialize_with_moveConstrained_follow_path(const moveConstrained_follow_path_task &task, const int motion_profile);
        int initialize_with_moveTo_follow_path(const moveTo_follow_path_task &task, const int motion_profile);
        int initialize_with_moveTo_weight_compensation(const moveTo_weight_compensation_task &task, const int motion_profile,
                                                       const Eigen::VectorXd &compensation_parameters);
        int initialize_with_moveTo(const moveTo_task &task, const int motion_profile);
        int initialize_with_moveGuarded(const moveGuarded_task &task, const int motion_profile);
        int initialize_with_full_pose(const full_pose_task &task, const int motion_profile);
        int initialize_with_gravity_compensation(const gravity_compensation_task &task);
        int update_weight_compensation_task_status(const int loop_iteration_count,
                                                   const Eigen::VectorXd &bias_signal,
                                                   const Eigen::VectorXd &gain_signal,
                                                   Eigen::VectorXd &filtered_bias);
        int update_force_task_status(const KDL::Wrench &desired_force, 
                                     const KDL::Wrench &ext_force,
                                     const double current_task_time,
                                     const double time_threshold);
        int update_motion_task_status(const state_specification &robot_state,
                                      state_specification &desired_state,
                                      const KDL::Twist &current_error,
                                      const KDL::Wrench &ext_force,
                                      const double time_passed_sec,
                                      const int tube_section_count);

    private:
        const int NUM_OF_JOINTS_, NUM_OF_SEGMENTS_, NUM_OF_FRAMES_, NUM_OF_CONSTRAINTS_;
        const int END_EFF_;
        int desired_task_model_, motion_profile_, loop_period_count_, compensator_trigger_count_;
        double total_control_time_sec_, previous_task_time_, total_contact_time_;
        bool goal_reached_, time_limit_reached_, contact_detected_, 
             contact_alignment_performed_, write_compensation_time_to_file_;
        Eigen::VectorXd filtered_bias_, compensation_parameters_;
        state_specification robot_state_, desired_state_;
        moving_variance variance_gain_, variance_bias_;
        moving_slope slope_bias_;
        KDL::Twist current_error_;
        KDL::Wrench ext_wrench_;
        moveTo_task moveTo_task_;
        moveGuarded_task moveGuarded_task_;
        moveTo_weight_compensation_task moveTo_weight_compensation_task_;
        full_pose_task full_pose_task_;
        gravity_compensation_task gravity_compensation_task_;
        moveTo_follow_path_task moveTo_follow_path_task_;
        moveConstrained_follow_path_task moveConstrained_follow_path_task_;
        std::ofstream log_file_ext_force_, log_file_compensation_;

        int update_full_pose_task(state_specification &desired_state);
        int update_gravity_compensation_task();
        int update_moveTo_task(state_specification &desired_state);
        int update_moveGuarded_task(state_specification &desired_state);
        int update_moveTo_weight_compensation_task(state_specification &desired_state);
        int update_moveTo_follow_path_task(state_specification &desired_state,
                                           const int tube_section_count);
        int update_moveConstrained_follow_path_task(state_specification &desired_state,
                                                    const int tube_section_count);
        bool contact_detected(const double linear_force_threshold, 
                              const double angular_force_threshold);
        bool contact_alignment_secured(const KDL::Wrench &desired_force,
                                       const KDL::Wrench &ext_force);
        bool force_goal_maintained(const KDL::Wrench &desired_force,
                                   const KDL::Wrench &ext_force);
        void low_pass_filter(const KDL::Wrench &ext_force, const double alpha);
        void low_pass_filter(const Eigen::VectorXd &signal, const double alpha);
        int sign(double x);
        void log_compensation_data(const int loop_iteration_count,
                                   const Eigen::VectorXd &bias_signal,
                                   const Eigen::VectorXd &gain_signal);
};
#endif /* FINITE_STATE_MACHINE_HPP */