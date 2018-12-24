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

#include <safety_controller.hpp>

safety_controller::safety_controller(
                            const KDL::Chain &chain,
                            const std::vector<double> joint_position_limits,
                            const std::vector<double> joint_velocity_limits,
                            const std::vector<double> joint_acceleration_limits,
                            const std::vector<double> joint_torque_limits):
        robot_chain_(chain),
        NUMBER_OF_JOINTS_(chain.getNrOfJoints()),
        NUMBER_OF_SEGMENTS_(chain.getNrOfSegments()),
        NUMBER_OF_FRAMES_(NUMBER_OF_SEGMENTS_ + 1),
        predictor_(robot_chain_),
        joint_position_limits_(joint_position_limits),
        joint_velocity_limits_(joint_velocity_limits),
        joint_acceleration_limits_(joint_acceleration_limits),
        joint_torque_limits_(joint_torque_limits){}

int safety_controller::check_limits(const state_specification &current_state,
                                    state_specification &commands,
                                    const double dt_sec)
{   
    predictor_.integrate_joint_space(current_state, commands, dt_sec, 1);

    //If joint limit reached, stop the program
    for (int i = 0; i < NUMBER_OF_JOINTS_; i++)
    {
        if (abs(commands.qd(i)) > joint_velocity_limits_[i])
        {
            std::cout << "Joint "<< i + 1 << "rate: " 
                      <<commands.qd(i) << " rad/s over limit" 
                      << std::endl;

            return -1;
        }
    }

    return -0;
}