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
#include <model_prediction.hpp>
#include <safety_controller.hpp>
#include <abag.hpp>
#include <constants.hpp>
#include <eigen_conversions/eigen_kdl.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <chrono>
#include <time.h>
#include <thread> 
#include <unistd.h> /*usleep function*/
#include <cmath>
#include <stdlib.h> /* abs */

enum task_interface
{
  CART_POSE,
  CART_VELOCITY,
  CART_ACCELERATION,
  CART_FORCE,
  FF_JOINT_TORQUE
};

class dynamics_controller
{
  public:
    dynamics_controller(youbot_mediator &robot_driver, const int rate_hz);
    ~dynamics_controller(){};

    int control(const int desired_control_mode, 
                const int desired_task_interface,
                const bool store_control_data);

    void reset_desired_state();

    /**
     * Method for defining desired robot pose. Interface exposed by this controller.
     * constraint_direction argument defines which of 6 DOF should be controlled.
     * Basically, some of DOFs can be left out to not be controlled by this 
     * controller, but left out to be controlled by natural dynamics of the 
     * system and environment.
     * First 3 elements of cartesian_pose vector define x, y, z positions
     * Last 3 elements define orientations around x, y, z axes - Roll, Pitch, Yaw 
     * All DOF are specified w.r.t. fixed (non-moving) robot base reference frame
     * 
     * Orientation convention (last 3 elements of cartesian_pose vector):
     * - First rotate around X with roll, then around the fixed Y with pitch, 
     *   then around fixed Z with yaw.
     * - Invariants:
     *   - RPY(roll, pitch, yaw) == RPY( roll +/- PI, PI - pitch, yaw +/- PI )
     *   - angles + 2*k*PI
     */
    void define_desired_ee_pose(const std::vector<bool> &constraint_direction,
                                const std::vector<double> &cartesian_pose);

    // Methods for defining robot task via 3 interfaces exposed by Vereshchagin
    void define_ee_acc_constraint(const std::vector<bool> &constraint_direction,
                                  const std::vector<double> &cartesian_acceleration);
    void define_ee_external_force(const std::vector<double> &external_force);
    void define_feadforward_torque(const std::vector<double> &ff_torque);

  private:
    const int RATE_HZ_;
    const long DT_MICRO_;
    const double DT_SEC_;
    int loop_count_;
    double loop_time_;

    int hd_solver_result_;
    int fk_solver_result_;
    int safe_control_mode_;

    const std::string LOG_FILE_PATH_;
    const Eigen::IOFormat WRITE_FORMAT_;
    std::ofstream log_file_;

    struct desired_control_mode
    {
      int interface;
      bool is_safe;
    } desired_control_mode_;

    std::chrono::steady_clock::time_point loop_start_time_;
    std::chrono::steady_clock::time_point loop_end_time_;
    std::chrono::duration <double, std::micro> loop_interval_{};

    const KDL::Chain robot_chain_;
    const int NUMBER_OF_JOINTS_;
    const int NUMBER_OF_SEGMENTS_;
    const int NUMBER_OF_FRAMES_;
    const int NUMBER_OF_CONSTRAINTS_;
    
    Eigen::VectorXd error_vector_;
    KDL::Rotation error_rot_matrix_;

    KDL::Solver_Vereshchagin hd_solver_;
    KDL::FK_Vereshchagin fk_vereshchagin_;
    safety_controller safety_control_;
    ABAG abag_;
    model_prediction predictor_;

    state_specification robot_state_;
    state_specification commands_;
    state_specification desired_state_;
    state_specification predicted_state_;

    void print_settings_info();
    void write_to_file(const Eigen::VectorXd &measured, const Eigen::VectorXd &desired);
    void reset_state(state_specification &state);
    void stop_robot_motion();
    void update_task();
    void update_current_state();
    void compute_control_error();
    void make_predictions();
    int apply_joint_control_commands();
    int evaluate_dynamics();
    int enforce_loop_frequency();

    void set_ee_acc_constraints(state_specification &state,
                                const std::vector<bool> &constraint_direction,
                                const std::vector<double> &cartesian_acceleration);
    void set_external_forces(state_specification &state, 
                             const std::vector<double> &external_force);
    void set_feadforward_torque(state_specification &state,
                                const std::vector<double> &ff_torque);    
};
#endif /* DYNAMICS_CONTROLLER_HPP_*/