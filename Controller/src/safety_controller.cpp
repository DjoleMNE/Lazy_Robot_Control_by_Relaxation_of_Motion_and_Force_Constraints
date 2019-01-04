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
                            const std::vector<double> joint_position_limits_p,
                            const std::vector<double> joint_position_limits_n,
                            const std::vector<double> joint_velocity_limits,
                            const std::vector<double> joint_acceleration_limits,
                            const std::vector<double> joint_torque_limits):
        robot_chain_(chain),
        NUMBER_OF_JOINTS_(chain.getNrOfJoints()),
        NUMBER_OF_SEGMENTS_(chain.getNrOfSegments()),
        NUMBER_OF_FRAMES_(chain.getNrOfSegments() + 1),
        predictor_(robot_chain_),
        joint_position_limits_p_(joint_position_limits_p),
        joint_position_limits_n_(joint_position_limits_n),
        joint_velocity_limits_(joint_velocity_limits),
        joint_acceleration_limits_(joint_acceleration_limits),
        joint_torque_limits_(joint_torque_limits),
        pos_limit_reached_(chain.getNrOfJoints(), true),
        print_logs_(true)
        {

        }
        
int safety_controller::generate_commands(const state_specification &current_state,
                                         state_specification &commands,
                                         const double dt_sec,
                                         const int desired_control_mode,
                                         const int prediction_method)
{   
    // Integrate joint acceleration to velocities and positions
    // I.e. generate initial commands
    predictor_.integrate_joint_space(current_state, commands, 
                                     dt_sec, 1, prediction_method);
    commands.control_torque = current_state.control_torque;
    commands.qdd = current_state.qdd;

    /* 
        First Safety Level: Is the Current State Safe?
        I.e. Check for NaN and ifinite values in commands variables.
        Stop the motion if some of the values are NaN or infine.
        And check if robot is already close to pos limits, 
        given the current (not integrated) angles.
        If yes, check if its safe to move that joint, 
        i.e. if the commanded torqes work in direction 
        in which the position limit is already reached stop the robot, 
        else continue (allowed to move away from the limit).
    */
    for (int i = 0; i < NUMBER_OF_JOINTS_; i++)
    {
        if(!std::isfinite(commands.control_torque(i)) || \
           !std::isfinite(commands.qdd(i)) || \
           !std::isfinite(commands.qd(i)) || \
           !std::isfinite(commands.q(i)) || \
           limits_reached(current_state, i) || \
           reaching_position_limits(current_state, i)){
               if (print_logs_) { std::cout << "First Level Safety: Stop robot" 
                                           << std::endl; }
               return control_mode::stop_motion;
           } 
    }

    /*
        Second Safety Level: Is the Future State Safe?
        I.e. Check if the commads from desired control interface will make robot
        go over the limits.
        If yes: switch to another mode, or scale down the commands,
        or stop the robot.
        If not: continue with the original commands.
    */
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

bool safety_controller::limits_reached(const state_specification &state,
                                       const int joint)
{
    // TODO: Add check for measured torques, if it makes sense? I dont think so
    // Following check is only if e.g. previous torque command introduced too
    // high velocity or moved robot over position limits
    if ((state.q(joint) >= joint_position_limits_p_[joint]) || \
        (state.q(joint) <= joint_position_limits_n_[joint]) || \
        (fabs(state.qd(joint)) >= joint_velocity_limits_[joint])){
            return true; 
    } else return false;
}

bool safety_controller::reaching_position_limits(const state_specification &state,
                                                 const int joint)
{
    if (state.q(joint) > 0.95 * joint_position_limits_p_[joint]){
        if(state.qd(joint) > 0.01) return true;
    } 
    else if (state.q(joint) < 0.95 * joint_position_limits_n_[joint]){
        if(state.qd(joint) < -0.01) return true;
    } 
    else return false;
}

int safety_controller::check_torques(const state_specification &current_state,
                  state_specification &commands,
                  const int desired_control_mode)
{
    for (int i = 0; i < NUMBER_OF_JOINTS_; i++)
    {
        if((fabs(commands.control_torque(i)) > joint_torque_limits_[i]) || \
           (fabs(commands.qdd(i)) > joint_acceleration_limits_[i]) )
        {
            if(print_logs_){
                std::cout << "Joint "<< i + 1 << " command: " 
                        << commands.control_torque(i) << " Nm is over the limit" 
                        << std::endl;
            }
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
        if (fabs(commands.qd(i)) > joint_velocity_limits_[i])
        {
            if (print_logs_){
                std::cout << "Joint "<< i + 1 << " command: " 
                          << commands.qd(i) << " rad/s is over the limit" 
                          << std::endl;
            }
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
        if ((commands.q(i) >= joint_position_limits_p_[i]) || \
             commands.q(i) <= joint_position_limits_n_[i])
        {
            if (print_logs_){
                std::cout << "Joint "<< i + 1 << " command: " 
                          <<commands.q(i) << " rad is over the limit" 
                          << std::endl;
            }
            return control_mode::stop_motion;
        }
    }
    return desired_control_mode;
}