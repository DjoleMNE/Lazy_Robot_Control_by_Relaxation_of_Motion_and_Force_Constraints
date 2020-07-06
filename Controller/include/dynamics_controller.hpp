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
#include <solver_recursive_newton_euler.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/utilities/svd_eigen_HH.hpp>
#include <state_specification.hpp>
#include <robot_mediator.hpp>
#include <geometry_utils.hpp>
#include <model_prediction.hpp>
#include <safety_monitor.hpp>
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

enum class error_source
{
    empty = 0,
    rne_solver = 1,
    fk_solver = 2,
    vereshchagin_solver = 3,
    weight_compensator = 4,
    joint_safety_monitor = 5,
    cartesian_safety_monitor = 6,
    fsm = 7,
    ext_wrench_estimation = 8
};

class dynamics_controller
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    dynamics_controller(robot_mediator *robot_driver, 
                        const int rate_hz,
                        const bool compensate_gravity);
    ~dynamics_controller(){};
    
    //Main control loop
    int control();

    /**
    * Perform single step of the control loop, given current robot joint state
    * Required for RTT's updateHook method
    */
    int step(const KDL::JntArray &q_input,
             const KDL::JntArray &qd_input,
             const KDL::Wrench &ext_force_torque, 
             KDL::JntArray &tau_output,
             const double time_passed_sec,
             const int main_loop_iteration,
             const int stop_loop_iteration,
             const bool stopping_behaviour_on);

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
                        const Eigen::VectorXd &null_space_parameters,
                        const Eigen::VectorXd &compensation_parameters,
                        const Eigen::VectorXd &stop_motion_error_alpha,
                        const Eigen::VectorXd &stop_motion_bias_threshold,
                        const Eigen::VectorXd &stop_motion_bias_step,
                        const Eigen::VectorXd &stop_motion_gain_threshold,
                        const Eigen::VectorXd &stop_motion_gain_step,
                        const Eigen::VectorXd &wrench_estimation_gain);
    int initialize(const int desired_control_mode, 
                   const int desired_dynamics_interface,
                   const int desired_motion_profile,
                   const bool store_control_data,
                   const bool use_estimated_external_wrench);
    void deinitialize();

    void engage_lock();
    int apply_joint_control_commands(const bool bypass_safeties);
    int monitor_joint_safety();
    int estimate_external_wrench(const KDL::JntArray &joint_position_measured,
                                 const KDL::JntArray &joint_velocity_measured,
                                 const KDL::JntArray &joint_torque_measured, 
                                 KDL::Wrench &ext_force_torque);
    void write_to_file();

    void reset_desired_state();
    void define_moveTo_task(const std::vector<bool> &constraint_direction,
                            const std::vector<double> &tube_start_position,
                            const std::vector<double> &tube_tolerances,
                            const double tube_speed,
                            const double contact_threshold_linear,
                            const double contact_threshold_angular,
                            const double task_time_limit_sec,
                            const bool control_null_space,
                            const double desired_null_space_angle,
                            std::vector<double> &task_frame_pose);

    void define_moveGuarded_task(const std::vector<bool> &constraint_direction,
                                 const std::vector<double> &tube_start_position,
                                 const std::vector<double> &tube_tolerances,
                                 const double tube_speed,
                                 const double contact_threshold_linear,
                                 const double contact_threshold_angular,
                                 const double task_time_limit_sec,
                                 const bool control_null_space,
                                 const double desired_null_space_angle,
                                 std::vector<double> &tube_end_position);

    void define_moveTo_weight_compensation_task(const std::vector<bool> &constraint_direction,
                                                const std::vector<double> &tube_start_position,
                                                const std::vector<double> &tube_tolerances,
                                                const double tube_speed,
                                                const double contact_threshold_linear,
                                                const double contact_threshold_angular,
                                                const double task_time_limit_sec,
                                                const bool control_null_space,
                                                const double desired_null_space_angle,
                                                const bool use_mass_alternation,
                                                std::vector<double> &task_frame_pose);

    void define_moveConstrained_follow_path_task(const std::vector<bool> &constraint_direction,
                                                 const std::vector< std::vector<double> > &tube_path_points,
                                                 const std::vector<double> &tube_tolerances,
                                                 const double tube_speed,
                                                 const double tube_force,
                                                 const double contact_threshold_linear,
                                                 const double contact_threshold_angular,
                                                 const double task_time_limit_sec,
                                                 const bool control_null_space,
                                                 const double desired_null_space_angle,
                                                 std::vector< std::vector<double> > &task_frame_poses);

    void define_moveTo_follow_path_task(const std::vector<bool> &constraint_direction,
                                        const std::vector< std::vector<double> > &tube_path_points,
                                        const std::vector<double> &tube_tolerances,
                                        const double tube_speed,
                                        const double contact_threshold_linear,
                                        const double contact_threshold_angular,
                                        const double task_time_limit_sec,
                                        const bool control_null_space,
                                        const double desired_null_space_angle,
                                        std::vector< std::vector<double> > &task_frame_poses);
    void define_full_pose_task(const std::vector<bool> &constraint_direction,
                               const std::vector<double> &cartesian_pose,
                               const double contact_threshold_linear,
                               const double contact_threshold_angular,
                               const double task_time_limit_sec,
                               const bool control_null_space,
                               const double desired_null_space_angle,
                               const double null_space_tolerance);

    void define_gravity_compensation_task(const double task_time_limit_sec);

    // Methods for defining robot task via 3 interfaces exposed by Vereshchagin
    void define_ee_acc_constraint(const std::vector<bool> &constraint_direction,
                                  const std::vector<double> &cartesian_acceleration);
    void define_ee_external_force(const std::vector<double> &external_force);
    void define_feedforward_torque(const std::vector<double> &ff_torque);

  private:
    const int RATE_HZ_;
    const long DT_MICRO_, DT_1KHZ_MICRO_, DT_STOPPING_MICRO_;
    const double DT_SEC_;

    std::ofstream log_file_cart_, log_file_joint_, log_file_predictions_, log_file_null_space_, log_file_cart_base_, log_file_stop_motion_, log_file_ext_wrench_;
    bool store_control_data_, use_estimated_external_wrench_;
    int desired_dynamics_interface_, desired_task_model_;

    struct desired_control_mode
    {
      int interface;
      bool is_safe;
    } desired_control_mode_;

    struct error_logger
    {
      error_source error_source_;
      int error_status_;
      int robot_id_;
    } error_logger_;

    std::chrono::steady_clock::time_point loop_start_time_;
    std::chrono::duration <double, std::micro> loop_interval_{};
    double total_time_sec_;
    int loop_iteration_count_, stop_loop_iteration_count_, steady_stop_iteration_count_,
        feedforward_loop_count_, control_loop_delay_count_;

    robot_mediator *robot_driver_;
    // Main chain is prepared for vereshchagin (nj == ns) but full contains additional segments
    KDL::Chain robot_chain_, robot_chain_full_;
    const int NUM_OF_JOINTS_;
    const int NUM_OF_SEGMENTS_;
    const int NUM_OF_FRAMES_;
    const int NUM_OF_CONSTRAINTS_;
    const int END_EFF_;
    const int ROBOT_ID_;
    const double INITIAL_END_EFF_MASS_;
    const bool COMPENSATE_GRAVITY_;
    const std::vector<double> JOINT_ACC_LIMITS_, JOINT_TORQUE_LIMITS_, JOINT_STOPPING_TORQUE_LIMITS_, JOINT_INERTIA_;
    const KDL::Twist ROOT_ACC_;
    std::vector<bool> CTRL_DIM_, POS_TUBE_DIM_, MOTION_CTRL_DIM_, FORCE_CTRL_DIM_;
    std::vector< std::deque<double> > stop_motion_setpoint_array_;
    int fsm_result_, fsm_force_task_result_, previous_task_status_, tube_section_count_;
    bool transform_drivers_, transform_force_drivers_, apply_feedforward_force_, 
         compute_null_space_command_, write_contact_time_to_file_,
         compensate_unknown_weight_, trigger_stopping_sequence_, stopping_sequence_on_;

    moveTo_task moveTo_task_;
    moveGuarded_task moveGuarded_task_;
    moveTo_weight_compensation_task moveTo_weight_compensation_task_;
    full_pose_task full_pose_task_;
    gravity_compensation_task gravity_compensation_task_;
    moveTo_follow_path_task moveTo_follow_path_task_;
    moveConstrained_follow_path_task moveConstrained_follow_path_task_;

    KDL::Twist current_error_twist_;
    Eigen::VectorXd abag_error_vector_, null_space_abag_error_, stop_motion_abag_error_, predicted_error_twist_, compensation_error_;
    double horizon_amplitude_, null_space_abag_command_, null_space_angle_, desired_null_space_angle_, updated_mass_estimation_;
    Eigen::VectorXd abag_command_, abag_stop_motion_command_, max_command_, compensation_parameters_, null_space_parameters_, force_task_parameters_, min_sat_limits_, filtered_bias_;
    KDL::Wrenches cart_force_command_, zero_wrenches_full_model_;
    KDL::Wrench ext_wrench_, ext_wrench_base_, compensated_weight_;
    KDL::JntArray zero_joint_array_, gravity_torque_, coriolis_torque_, estimated_ext_torque_, filtered_estimated_ext_torque_, estimated_momentum_integral_, initial_jnt_momentum_;
    KDL::JntSpaceInertiaMatrix jnt_mass_matrix_, previous_jnt_mass_matrix_, jnt_mass_matrix_dot_;
    KDL::Jacobian jacobian_end_eff_;
    Eigen::MatrixXd svd_U_, svd_V_, jacobian_end_eff_inv_, jacobian_end_eff_inv_temp_;
    Eigen::VectorXd svd_S_, svd_tmp_, wrench_estimation_gain_;

    KDL::FK_Vereshchagin fk_vereshchagin_;
    KDL::ChainJntToJacSolver jacobian_solver_;
    safety_monitor safety_monitor_;
    ABAG abag_, abag_null_space_, abag_stop_motion_;
    finite_state_machine fsm_;
    model_prediction predictor_;

    state_specification robot_state_, robot_state_base_;
    state_specification desired_state_, desired_state_base_;
    state_specification predicted_state_;
    std::vector<state_specification> predicted_states_; 

    const Eigen::IOFormat WRITE_FORMAT_STOP_MOTION;

    std::shared_ptr<KDL::Solver_Vereshchagin> hd_solver_;
    std::shared_ptr<KDL::Solver_RNE> id_solver_;
    std::shared_ptr<KDL::ChainDynParam> dynamic_parameter_solver_;

    int update_commands(); //Performs single update of control commands and dynamics computations
    int update_stop_motion_commands();
    int check_fsm_status();
    int update_current_state();
    void print_settings_info();
    void close_files();
    void reset_state(state_specification &state);
    // void update_dynamics_interfaces();
    void compute_moveConstrained_follow_path_task_error();
    void compute_moveConstrained_null_space_task_error();
    void compute_moveToGuarded_null_space_task_error();
    void compute_moveTo_follow_path_task_error();
    void compute_moveTo_task_error();
    void compute_moveGuarded_task_error();
    void compute_moveTo_weight_compensation_task_error();
    void compute_full_pose_task_error();
    void compute_gravity_compensation_task_error();
    void compute_control_error();
    void transform_force_driver();
    void transform_motion_driver();
    void make_Cartesian_predictions(const double dt_sec, const int num_steps);
    void compute_cart_control_commands();
    void compute_null_space_control_commands();
    int compute_weight_compensation_control_commands();
    KDL::Twist finite_displacement_twist(const state_specification &state_a, 
                                         const state_specification &state_b);
    double kinetic_energy(const KDL::Twist &twist, const int segment_index);
    int evaluate_dynamics();
    int compute_gravity_compensation_control_commands();
    int enforce_loop_frequency(const int dt);

    void set_ee_acc_constraints(state_specification &state,
                                const std::vector<bool> &constraint_direction,
                                const std::vector<double> &cartesian_acceleration);
    void set_external_forces(state_specification &state, 
                             const std::vector<double> &external_force);
    void set_feedforward_torque(state_specification &state,
                                const std::vector<double> &ff_torque);    
};
#endif /* DYNAMICS_CONTROLLER_HPP_*/