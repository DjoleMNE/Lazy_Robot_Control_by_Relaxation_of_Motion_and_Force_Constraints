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

#ifndef SAFETY_MONITOR_HPP_
#define SAFETY_MONITOR_HPP_
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

class safety_monitor
{
  public:
    safety_monitor(robot_mediator *robot_driver, const bool print_logs);
    ~safety_monitor(){}

    int monitor_joint_state(const state_specification &current_state,
                            const double dt_sec,
                            const int desired_control_mode,
                            const std::vector<state_specification> &predicted_states);

  private:
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

    state_specification current_state_;
    std::vector<state_specification> predicted_states_; 

    bool is_current_state_safe();
    bool is_state_finite(const state_specification &state, const int joint);
    bool torque_limit_reached(const state_specification &state, const int joint);
    bool velocity_limit_reached(const state_specification &state, const int joint);
    bool position_limit_reached(const state_specification &state, const int joint);
    bool reaching_position_limits(const state_specification &state, const int joint);
    
    int monitor_future_state(const int desired_control_mode);
};
#endif /* SAFETY_MONITOR_HPP_*/