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
                            const std::vector<double> joint_torque_limits,
                            const bool print_logs):
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
        pos_limits_checked_(false),
        vel_limits_checked_(false),
        pos_limit_reached_(chain.getNrOfJoints(), false),
        vel_limit_reached_(chain.getNrOfJoints(), false),
        print_logs_(print_logs)
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
        I.e. Check for NaN and infinite values in the commands variables.
        Stop the motion if some of the values are NaN or infinite.
        And check if robot has already crossed the position and velocity limits.
        Also check if robot is close to position limits, 
        given the current (not integrated) angles and velocities.
        If everything ok, proceed to the second level.
    */
    if (!is_current_state_safe(current_state, commands)) 
        return control_mode::stop_motion;

    /*
        Second Safety Level: Is the Future State Safe?
        I.e. Check if the commads from desired control interface will make robot
        go over the limits.
        If yes: switch to another mode, and/or scale down the commands,
        or stop the robot.
        If not: continue with the original commands.
    */
    // pos_limits_checked_ = false;
    // vel_limits_checked_ = false;
    // std::fill(pos_limit_reached_.begin(), pos_limit_reached_.end(), false);
    // std::fill(vel_limit_reached_.begin(), pos_limit_reached_.end(), false);
    return check_future_state(commands, desired_control_mode);
}

bool safety_controller::is_current_state_safe(
                                        const state_specification &robot_state,
                                        const state_specification &commands)
{
    for (int i = 0; i < NUMBER_OF_JOINTS_ - 1; i++)
    {
        /* 
            Check current velocities and position for every case. 
            Our model is not correct and commanded torque in previous iteration,
            may produce too high velocities or move joint over position limits.
            Basically check for error in model prediction.
            TODO: Investigate if its necessary also to check for mesured torques.
            It depends on:
                 - which torque we command, full or constraint
                 - what sensor readings we receive: full torqe on the joint, 
                                             or just torque produced by motors
        */
        if(!is_finite(commands, i) || \
            /*
           velocity_limit_reached(robot_state, i) || \
           position_limit_reached(robot_state, i) || \
           */
           reaching_position_limits(robot_state, i)){
                if (print_logs_) 
                    std::cout << "Current robot state is not safe" << std::endl;
               return false;
           } 
    }
    return true;
}

int safety_controller::check_future_state(state_specification &commands, 
                                          const int desired_control_mode)
{
    switch (desired_control_mode)
    {   
        case control_mode::torque_control:
            return check_torques(commands, desired_control_mode);

        case control_mode::velocity_control:
            return check_velocities(commands, desired_control_mode, 0);

        case control_mode::position_control:
            return check_positions(commands, desired_control_mode, 0);
    
        default: return control_mode::stop_motion;
    }
}

bool safety_controller::is_finite(const state_specification &commands, 
                                  const int joint)
{
    if (!std::isfinite(commands.control_torque(joint))){
        if (print_logs_) 
            std::cout << "Computed torque for joint: "<< joint + 1 
                      <<" is not finite!" << std::endl;
        return false;
    }
    else if (!std::isfinite(commands.qdd(joint))){
        if (print_logs_) 
            std::cout << "Computed joint "<< joint + 1 
                      <<" acceleration is not finite!" << std::endl;
        return false;
    }
    else if (!std::isfinite(commands.qd(joint))){
        if (print_logs_) 
            std::cout << "Computed joint "<< joint + 1 
                      <<" velocity is not finite!" << std::endl;
        return false;
    }
    else if (!std::isfinite(commands.q(joint))){
        if (print_logs_) 
            std::cout << "Computed joint "<< joint + 1
                      <<" position is not finite!" << std::endl;
        return false;
    }
    else return true;
}

bool safety_controller::torque_limit_reached(const state_specification &command,
                                             const int joint)
{
    if (fabs(command.control_torque(joint)) >= joint_velocity_limits_[joint])
    {
        if(print_logs_) 
            std::cout << "Joint " << joint + 1 
                      << " torque limit reached!" << std::endl;
        return true;        
    } else return false;
}

bool safety_controller::velocity_limit_reached(const state_specification &state,
                                               const int joint)
{
    if (fabs(state.qd(joint)) >= joint_velocity_limits_[joint])
    {
        if(print_logs_) 
            std::cout << "Joint " << joint + 1 
                      << " velocity limit reached!" << std::endl;
        return true;        
    } else return false;
}

bool safety_controller::position_limit_reached(const state_specification &state,
                                               const int joint)
{
    if ((state.q(joint) >= joint_position_limits_p_[joint]) || \
        (state.q(joint) <= joint_position_limits_n_[joint]))
    {
        if(print_logs_) 
            std::cout << "Joint " << joint + 1 
                      << " position limit reached!" << std::endl;
        return true; 
    } else return false;
}

bool safety_controller::reaching_position_limits(const state_specification &state,
                                                 const int joint)
{
    // TODO: make this percentage different for each joint!
    // Bigger space for joint to move, bigger the percentage!
    if (state.q(joint) > 0.95 * joint_position_limits_p_[joint]){
        if(state.qd(joint) > 0.01) 
            std::cout << "Joint " << joint + 1 << " is too close to the limit"
                      << std::endl;
        return true;
    } 
    else if (state.q(joint) < 0.95 * joint_position_limits_n_[joint]){
        if(state.qd(joint) < -0.01) 
            std::cout << "Joint " << joint + 1 << " is too close to the limit"
                      << std::endl;
        return true;
    } 
    else return false;
}

/*
    Check if the commaned torques are over torque limits.
    Check if the commanded torqes will make a joint go over the limits.
    If all ok: continue with this control mode.
    Else: switch to the velocity mode.
*/
int safety_controller::check_torques(state_specification &commands,
                                     const int desired_control_mode)
{

    for (int i = 0; i < NUMBER_OF_JOINTS_ - 1; i++)
    {
        if(torque_limit_reached(commands, i) || \
           velocity_limit_reached(commands, i) || \
           position_limit_reached(commands, i))
            // Switch to the velocity control mode
            return check_velocities(commands, desired_control_mode, i);

    } return desired_control_mode;
}

int safety_controller::check_velocities(state_specification &commands,
                                        const int desired_control_mode,
                                        const int start_index)
{
    //If a velocity limit reached, rescale commands or stop the program
    for (int i = start_index; i < NUMBER_OF_JOINTS_; i++)
    {
        if (velocity_limit_reached(commands, i))
        {
            return control_mode::position_control;
        }
    }
    return desired_control_mode;
}

/*
    Simple check if the computed postion commands are valid, i.e. over the limit.
    Last step in safety check.
*/
int safety_controller::check_positions(state_specification &commands,
                                       const int desired_control_mode,
                                       const int start_index)
{
    //If a position limit reached, stop the program
    for (int i = 0; i < NUMBER_OF_JOINTS_; i++)
    {
        if (position_limit_reached(commands, i))
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