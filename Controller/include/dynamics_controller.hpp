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

#ifndef DYNAMICS_CONTROLLER_HPP_
#define DYNAMICS_CONTROLLER_HPP_
#include <solver_vereshchagin.hpp>
#include <fk_vereshchagin.hpp>
#include <state_specification.hpp>
#include <robot_mediator.hpp>
#include <geometry_utils.hpp>
#include <model_prediction.hpp>
#include <safety_controller.hpp>
#include <finite_state_machine.hpp>
#include <motion_profile.hpp>
#include <utility> 
#include <abag.hpp>
#include <constants.hpp>
#include <kdl_eigen_conversions.hpp>
#include <iostream>
#include <sstream>
#include <fstream>
#include <chrono>
#include <time.h>
#include <thread> 
#include <unistd.h> /*usleep function*/
#include <cmath>
#include <stdlib.h> /* abs */

enum dynamics_interface
{
  CART_ACCELERATION = 0,
  CART_FORCE = 1,
  FF_JOINT_TORQUE = 2
};

class dynamics_controller
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    dynamics_controller(robot_mediator *robot_driver, const int rate_hz);
    ~dynamics_controller(){};
    
    //Main control loop
    int control(const int desired_control_mode, const bool store_control_data);

    /**
    * Perform single step of the control loop, given current robot joint state
    * Required for RTT's updateHook method
    */
    int step(const KDL::JntArray &q_input,
             const KDL::JntArray &qd_input,
             const KDL::Wrench &ext_force, 
             Eigen::VectorXd &tau_output,
             const double time_passed_sec,
             const int loop_iteration);
    
    void set_parameters(const double damper_amplitude,
                        const int abag_error_type,
                        const Eigen::VectorXd &max_command,
                        const Eigen::VectorXd &error_alpha, 
                        const Eigen::VectorXd &bias_threshold, 
                        const Eigen::VectorXd &bias_step, 
                        const Eigen::VectorXd &gain_threshold, 
                        const Eigen::VectorXd &gain_step,
                        const Eigen::VectorXd &min_bias_sat,
                        const Eigen::VectorXd &min_command_sat,
                        const Eigen::VectorXd &null_space_abag_parameters,
                        const Eigen::VectorXd &compensation_parameters);
    int initialize(const int desired_control_mode, 
                   const int desired_task_inteface,
                   const bool use_mixed_driver, 
                   const bool store_control_data,
                   const int motion_profile);
    void deinitialize();
    void stop_robot_motion();

    void reset_desired_state();
    void define_moveTo_task(const std::vector<bool> &constraint_direction,
                            const std::vector<double> &tube_start_position,
                            const std::vector<double> &tube_tolerances,
                            const double tube_speed,
                            const double contact_threshold_linear,
                            const double contact_threshold_angular,
                            const double task_time_limit_sec,
                            std::vector<double> &task_frame_pose);

    void define_moveTo_weight_compensation_task(const std::vector<bool> &constraint_direction,
                                                const std::vector<double> &tube_start_position,
                                                const std::vector<double> &tube_tolerances,
                                                const double tube_speed,
                                                const double contact_threshold_linear,
                                                const double contact_threshold_angular,
                                                const double task_time_limit_sec,
                                                std::vector<double> &task_frame_pose);

    void define_moveConstrained_follow_path_task(const std::vector<bool> &constraint_direction,
                                                 const std::vector< std::vector<double> > &tube_path_points,
                                                 const std::vector<double> &tube_tolerances,
                                                 const double tube_speed,
                                                 const double tube_force,
                                                 const double contact_threshold_linear,
                                                 const double contact_threshold_angular,
                                                 const double task_time_limit_sec,
                                                 std::vector< std::vector<double> > &task_frame_poses);

    void define_moveTo_follow_path_task(const std::vector<bool> &constraint_direction,
                                        const std::vector< std::vector<double> > &tube_path_points,
                                        const std::vector<double> &tube_tolerances,
                                        const double tube_speed,
                                        const double contact_threshold_linear,
                                        const double contact_threshold_angular,
                                        const double task_time_limit_sec,
                                        std::vector< std::vector<double> > &task_frame_poses);
    void define_desired_ee_pose(const std::vector<bool> &constraint_direction,
                                const std::vector<double> &cartesian_pose,
                                const double contact_threshold_linear,
                                const double contact_threshold_angular,
                                const double task_time_limit_sec);

    // Methods for defining robot task via 3 interfaces exposed by Vereshchagin
    void define_ee_acc_constraint(const std::vector<bool> &constraint_direction,
                                  const std::vector<double> &cartesian_acceleration);
    void define_ee_external_force(const std::vector<double> &external_force);
    void define_feedforward_torque(const std::vector<double> &ff_torque);

  private:
    const int RATE_HZ_;
    const long DT_MICRO_;
    const double DT_SEC_;

    std::ofstream log_file_cart_, log_file_joint_, log_file_predictions_, log_file_null_space_;
    bool store_control_data_;

    struct desired_control_mode
    {
      int interface;
      bool is_safe;
    } desired_control_mode_;

    int desired_task_inteface_, desired_task_model_;

    std::chrono::steady_clock::time_point loop_start_time_;
    std::chrono::steady_clock::time_point loop_end_time_;
    std::chrono::duration <double, std::micro> loop_interval_{};
    double total_time_sec_;
    int loop_iteration_count_, feedforward_loop_count_;

    const KDL::Chain robot_chain_;
    const int NUM_OF_JOINTS_;
    const int NUM_OF_SEGMENTS_;
    const int NUM_OF_FRAMES_;
    const int NUM_OF_CONSTRAINTS_;
    const int END_EFF_;
    const std::vector<double> JOINT_TORQUE_LIMITS_;
    std::vector<bool> CTRL_DIM_, POS_TUBE_DIM_, MOTION_CTRL_DIM_, FORCE_CTRL_DIM_;
    int fsm_result_, fsm_force_task_result_, previous_control_status_, tube_section_count_;
    bool transform_drivers_, transform_force_drivers_, apply_feedforward_force_, 
         compute_null_space_command_, write_contact_time_to_file_,
         compensate_unknown_weight_;
    
    moveTo_task moveTo_task_;
    moveTo_weight_compensation_task moveTo_weight_compensation_task_;
    full_pose_task full_pose_task_;
    moveTo_follow_path_task moveTo_follow_path_task_;
    moveConstrained_follow_path_task moveConstrained_follow_path_task_;

    KDL::Twist current_error_twist_;
    Eigen::VectorXd abag_error_vector_, null_space_abag_error_, 
                    predicted_error_twist_, compensation_error_;
    double horizon_amplitude_, null_space_abag_command_, null_space_angle_;
    Eigen::VectorXd abag_command_, max_command_, compensation_parameters_,
                    force_task_parameters_, min_sat_limits_, filtered_bias_;
    KDL::Wrenches cart_force_command_;
    KDL::Wrench ext_wrench_;

    KDL::Solver_Vereshchagin hd_solver_;
    KDL::FK_Vereshchagin fk_vereshchagin_;
    safety_controller safety_control_;
    ABAG abag_, abag_null_space_;
    finite_state_machine fsm_;
    model_prediction predictor_;

    state_specification robot_state_;
    state_specification desired_state_;
    state_specification predicted_state_;

    int check_control_status();
    void print_settings_info();
    void write_to_file();
    void reset_state(state_specification &state);
    void update_dynamics_interfaces();
    void update_current_state();
    void compute_moveConstrained_follow_path_task_error();
    void compute_moveConstrained_null_space_task_error();
    void compute_moveTo_follow_path_task_error();
    void compute_moveTo_task_error();
    void compute_moveTo_weight_compensation_task_error();
    void compute_full_pose_task_error();
    void compute_control_error();
    void transform_force_driver();
    void transform_motion_driver();
    void make_predictions(const double dt_sec, const int num_steps);
    void compute_cart_control_commands();
    void compute_weight_compensation_control_commands();
    KDL::Twist finite_displacement_twist(const state_specification &state_a, 
                                         const state_specification &state_b);
    double kinetic_energy(const KDL::Twist &twist, const int segment_index);
    int apply_joint_control_commands();
    int evaluate_dynamics();
    int enforce_loop_frequency();

    void set_ee_acc_constraints(state_specification &state,
                                const std::vector<bool> &constraint_direction,
                                const std::vector<double> &cartesian_acceleration);
    void set_external_forces(state_specification &state, 
                             const std::vector<double> &external_force);
    void set_feedforward_torque(state_specification &state,
                                const std::vector<double> &ff_torque);    
};
#endif /* DYNAMICS_CONTROLLER_HPP_*/