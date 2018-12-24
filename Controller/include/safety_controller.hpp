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
#include <model_prediction.hpp>
#include <iostream>
#include <sstream>
#include <time.h>
#include <fstream>
#include <cmath>
#include <stdlib.h>     /* abs */

enum control_mode 
{
    torque_control = 0,
    velocity_control = 1,
    position_control = 2, 
    stop_motion = -1   
};

class safety_controller
{
  public:
    safety_controller(const KDL::Chain &chain,
                      const std::vector<double> joint_position_limits,
                      const std::vector<double> joint_velocity_limits,
                      const std::vector<double> joint_acceleration_limits,
                      const std::vector<double> joint_torque_limits);
    ~safety_controller(){};

    int check_limits(const state_specification &current_state,
                     state_specification &commands,
                     const double dt_sec,
                     const int desired_control_mode);

  private:

    const int NUMBER_OF_JOINTS_;
    const int NUMBER_OF_SEGMENTS_;
    const int NUMBER_OF_FRAMES_;
        
    const KDL::Chain robot_chain_;

    const std::vector<double> joint_position_limits_;
    const std::vector<double> joint_velocity_limits_;
    const std::vector<double> joint_acceleration_limits_;
    const std::vector<double> joint_torque_limits_;

	model_prediction predictor_;

    // state_specification robot_state_;
    // state_specification commands_;
    // state_specification predicted_state_;

    void reset_state(state_specification &state);
    void reduce_velocities(){};
};
#endif /* SAFETY_CONTROLLER_HPP_*/