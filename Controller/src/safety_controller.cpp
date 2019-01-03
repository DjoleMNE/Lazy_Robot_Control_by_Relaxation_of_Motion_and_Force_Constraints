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
                            const std::vector<double> joint_position_limits_l,
                            const std::vector<double> joint_position_limits_r,
                            const std::vector<double> joint_velocity_limits,
                            const std::vector<double> joint_acceleration_limits,
                            const std::vector<double> joint_torque_limits):
        robot_chain_(chain),
        NUMBER_OF_JOINTS_(chain.getNrOfJoints()),
        NUMBER_OF_SEGMENTS_(chain.getNrOfSegments()),
        NUMBER_OF_FRAMES_(NUMBER_OF_SEGMENTS_ + 1),
        predictor_(robot_chain_),
        joint_position_limits_l_(joint_position_limits_l),
        joint_position_limits_r_(joint_position_limits_r),
        joint_velocity_limits_(joint_velocity_limits),
        joint_acceleration_limits_(joint_acceleration_limits),
        joint_torque_limits_(joint_torque_limits)
        {

        }
        
int safety_controller::check_limits(const state_specification &current_state,
                                    state_specification &commands,
                                    const double dt_sec,
                                    const int desired_control_mode,
                                    const int prediction_method)
{   
    predictor_.integrate_joint_space(current_state, commands, 
                                     dt_sec, 1, prediction_method);
    
    switch (desired_control_mode)
    {   
        case control_mode::torque_control:
            return check_torques(current_state, commands, desired_control_mode);

        case control_mode::velocity_control:
            return check_velocities(current_state, commands, desired_control_mode);

        case control_mode::position_control:
            return check_positions(current_state, commands, desired_control_mode);
    
        default: return control_mode::stop_motion;
    }
}

int safety_controller::check_torques(const state_specification &current_state,
                  state_specification &commands,
                  const int desired_control_mode)
{
    commands.control_torque = current_state.control_torque;
    for (int i = 0; i < NUMBER_OF_JOINTS_; i++)
    {
        //Stop the motion if some of the values are NaN or infine
        if(!std::isfinite(commands.control_torque(i))) 
            return control_mode::stop_motion;

        else if(abs(commands.control_torque(i)) > joint_torque_limits_[i])
        {
            std::cout << "Joint "<< i + 1 << " : " 
                    <<commands.control_torque(i) << " Nm is over the limit" 
                    << std::endl;
            return control_mode::stop_motion;
        }
    }
    return desired_control_mode;
}

int safety_controller::check_velocities(const state_specification &current_state,
                     state_specification &commands,
                     const int desired_control_mode)
{
    //If a velocity limit reached, rescale commands or stop the program
    for (int i = 0; i < NUMBER_OF_JOINTS_; i++)
    {
        //Stop the motion if some of the values are NaN or infine
        if(!std::isfinite(commands.qd(i))) 
            return control_mode::stop_motion;

        if (abs(commands.qd(i)) > joint_velocity_limits_[i])
        {
            std::cout << "Joint "<< i + 1 << " : " 
                    <<commands.qd(i) << " rad/s is over the limit" 
                    << std::endl;
            return control_mode::stop_motion;
        }
    }
    return desired_control_mode;
}

int safety_controller::check_positions(const state_specification &current_state,
                    state_specification &commands,
                    const int desired_control_mode)
{
    //If a position limit reached, stop the program
    for (int i = 0; i < NUMBER_OF_JOINTS_; i++)
    {
        //Stop the motion if some of the values are NaN or infine
        if(!std::isfinite(commands.q(i))) 
            return control_mode::stop_motion;
        
        if ((commands.q(i) > joint_position_limits_l_[i]) \
            || commands.q(i) < joint_position_limits_r_[i])
        {
            std::cout << "Joint "<< i + 1 << " : " 
                    <<commands.q(i) << " rad is over the limit" 
                    << std::endl;
            return control_mode::stop_motion;
        }
    }
    return desired_control_mode;
}