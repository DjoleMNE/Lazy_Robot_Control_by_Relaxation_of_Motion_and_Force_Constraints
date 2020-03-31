/*
Author(s): Djordje Vukcevic, Sven Schneider
Institute: Hochschule Bonn-Rhein-Sieg

Copyright (c) [2018]

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

#ifndef SAFETY_CONTROLLER_HPP_
#define SAFETY_CONTROLLER_HPP_
#include <solver_vereshchagin.hpp>
#include <state_specification.hpp>
#include <robot_mediator.hpp>
#include <model_prediction.hpp>
#include <constants.hpp>
#include <iostream>
#include <sstream>
#include <fstream>
#include <cmath>
#include <math.h>       /* fabs */
#include <stdlib.h>     /* abs */

class safety_controller
{
  public:
    safety_controller(robot_mediator *robot_driver, const bool print_logs);
    ~safety_controller(){}

    int set_control_commands(const state_specification &current_state,
                             const double dt_sec,
                             const int desired_control_mode,
                             const int prediction_method,
                             const bool bypass_safeties);

    void get_control_commands(state_specification &commands);
    void get_current_state(state_specification &current_state);
    void stop_robot_motion();

  private:
    robot_mediator *robot_driver_;
    const KDL::Chain robot_chain_;
	  model_prediction predictor_;

    const std::vector<double> joint_position_limits_max_;
    const std::vector<double> joint_position_limits_min_;
    const std::vector<double> joint_position_thresholds_;
    const std::vector<double> joint_velocity_limits_;
    const std::vector<double> joint_torque_limits_;
    
    const int NUM_OF_JOINTS_;
    const int NUM_OF_SEGMENTS_;
    const int NUM_OF_FRAMES_;
    const int NUM_OF_CONSTRAINTS_;
    const bool PRINT_LOGS_;
    
    const KDL::JntArray zero_joint_velocities_;

    state_specification commands_;
    std::vector<state_specification> predicted_states_; 

    bool is_current_state_safe(const state_specification &current_state);
    bool is_state_finite(const state_specification &state, 
                         const int joint);
    bool torque_limit_reached(const state_specification &state,
                              const int joint);
    bool velocity_limit_reached(const state_specification &state,
                                const int joint);
    bool position_limit_reached(const state_specification &state,
                                const int joint);
    bool reaching_position_limits(const state_specification &state,
                                  const int joint);
    
    void generate_commands(const state_specification &current_state);

    int check_future_state(const int desired_control_mode);    
    int check_torques();
    int check_velocities();
    int check_positions();

    void make_predictions(const state_specification &current_state,
                          const double dt_sec, const int prediction_method);

    //Not implemented currently
    bool reduce_velocities();
};
#endif /* SAFETY_CONTROLLER_HPP_*/