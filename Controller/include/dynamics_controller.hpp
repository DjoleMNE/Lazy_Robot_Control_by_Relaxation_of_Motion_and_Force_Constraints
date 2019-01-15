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
#include <state_specification.hpp>
#include <model_prediction.hpp>
#include <safety_controller.hpp>
#include <abag.hpp>
#include <controller_constants.hpp>
#include <kdl_conversions/kdl_msg.h>
#include <eigen_conversions/eigen_kdl.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <chrono>
#include <time.h>
#include <thread> 
#include <unistd.h> /*usleep function*/
#include <cmath>
#include <stdlib.h>     /* abs */

enum task_interface
{
  CART_POSITION = 0,
  CART_VELOCITY = 1,
  CART_ACCELERATION = 2,
  CART_FORCE = 3,
  FF_JOINT_TORQUE = 4
};

class dynamics_controller
{
  public:
    dynamics_controller(youbot_mediator &robot_driver, const int rate_hz);
    ~dynamics_controller(){};

    int control(const int desired_control_mode, const int desired_task_interface);

    void reset_desired_state();

    //Methods for defining robot task via 3 interfaces
    void define_ee_constraint_task(const std::vector<bool> constraint_direction,
                                   const std::vector<double> cartesian_acceleration);
    void define_ee_external_force_task(const std::vector<double> external_force);
    void define_feadforward_torque_task(const std::vector<double> ff_torque);

  private:
    const int RATE_HZ_;
    const long DT_MICRO_;
    const double DT_SEC_;
    int solver_result_;
    int safe_control_mode_;

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
    
    KDL::Solver_Vereshchagin hd_solver_;
    safety_controller safety_control_;
    ABAG abag_;
    model_prediction predictor_;

    state_specification robot_state_;
    state_specification commands_;
    state_specification desired_state_;
    state_specification predicted_state_;

    void print_settings_info();
    void reset_state(state_specification &state);
    void stop_robot_motion();
    void update_task();
    void update_current_state();
    void make_predictions(const int prediction_method);
    int apply_control_commands();
    int evaluate_dynamics();
    int enforce_loop_frequency();

    void set_ee_constraints(state_specification &state,
                            const std::vector<bool> constraint_direction,
                            const std::vector<double> cartesian_acceleration);
    void set_external_forces(state_specification &state, 
                             const std::vector<double> external_force);
    void set_feadforward_torque(state_specification &state,
                                const std::vector<double> ff_torque);    
};
#endif /* DYNAMICS_CONTROLLER_HPP_*/