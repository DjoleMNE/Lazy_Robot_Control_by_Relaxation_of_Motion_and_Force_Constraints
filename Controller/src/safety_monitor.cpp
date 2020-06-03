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

#include <safety_monitor.hpp>

safety_monitor::safety_monitor(robot_mediator *robot_driver, const bool print_logs):
    joint_position_limits_max_(robot_driver->get_maximum_joint_pos_limits()),
    joint_position_limits_min_(robot_driver->get_minimum_joint_pos_limits()),
    joint_position_thresholds_(robot_driver->get_joint_position_thresholds()),
    joint_velocity_limits_(robot_driver->get_joint_velocity_limits()),
    joint_torque_limits_(robot_driver->get_joint_torque_limits()),
    NUM_OF_JOINTS_(robot_driver->get_robot_model().getNrOfJoints()),
    NUM_OF_SEGMENTS_(robot_driver->get_robot_model().getNrOfSegments()),
    NUM_OF_FRAMES_(robot_driver->get_robot_model().getNrOfSegments() + 1),
    NUM_OF_CONSTRAINTS_(dynamics_parameter::NUMBER_OF_CONSTRAINTS),
    PRINT_LOGS_(print_logs),
    current_state_(NUM_OF_JOINTS_, NUM_OF_SEGMENTS_, NUM_OF_FRAMES_, NUM_OF_CONSTRAINTS_),
    predicted_states_(2, current_state_)
{

}

int safety_monitor::monitor_joint_state(const state_specification &current_state,
                                           const double dt_sec,
                                           const int desired_control_mode,
                                           const std::vector<state_specification> &predicted_states)
{
    assert(NUM_OF_JOINTS_ == current_state.qd.rows());
    current_state_ = current_state;

    /*
        First Safety Level: Is the Current State Safe?
        I.e. Check for NaN and infinite values in the commands variables.
        Stop the motion if some of the values are NaN or infinite.
        And check if robot has already crossed the position and velocity limits.
        Also check if robot is close to position limits,
        given the measured (not integrated) angles and velocities.
        If everything ok, proceed to the second level.
    */
    if (!is_current_state_safe()) return control_mode::STOP_MOTION;

    /*
        Integrated joint accelerations to velocities and positions
        I.e. prediction on where the robot will end-up
        in next two steps, if the computed commands have been applied.
    */
    predicted_states_ = predicted_states;

    /*
        Second Safety Level: Is the Future State Safe?
        I.e. Check if the commads from desired control interface will
        make robot go over the limits.
        If yes, stop the robot.
        If not: continue with the original commands.
    */
    return monitor_future_state(desired_control_mode);
}

bool safety_monitor::is_current_state_safe()
{
    /* 
        Check current velocities and position for every case. 
        Our model is not correct and commanded torque in previous iteration,
        may produce too high velocities or move joint over position limits.
        Basically check for error in model evaluation and predictions.
    */
    for (int i = 0; i < NUM_OF_JOINTS_; i++)
    {
        if (!is_state_finite(current_state_, i) || \
            velocity_limit_reached(current_state_, i) || \
            position_limit_reached(current_state_, i) || \
            reaching_position_limits(current_state_, i)) return false;
    }
    return true;
}

bool safety_monitor::is_state_finite(const state_specification &state, const int joint)
{
    if (!std::isfinite(state.control_torque(joint))){
        if (PRINT_LOGS_) printf("Computed torque for joint: %d is not finite!\n", joint + 1);
        return false;
    }
    else if (!std::isfinite(state.qdd(joint))){
        if (PRINT_LOGS_) printf("Computed joint: %d acceleration is not finite!\n", joint + 1);
        return false;
    }
    else if (!std::isfinite(state.qd(joint))){
        if (PRINT_LOGS_) printf("Computed joint: %d velocity is not finite!\n", joint + 1);
        return false;
    }
    else if (!std::isfinite(state.q(joint))){
        if (PRINT_LOGS_) printf("Computed joint: %d position is not finite!\n", joint + 1);
        return false;
    }

    return true;
}

bool safety_monitor::torque_limit_reached(const state_specification &state, const int joint)
{
    if (std::fabs(state.control_torque(joint)) >= joint_torque_limits_[joint])
    {
        if (PRINT_LOGS_) printf("Joint %d torque limit reached: %f \n", joint + 1, state.control_torque(joint));
        return true;        
    }

    return false;
}

bool safety_monitor::velocity_limit_reached(const state_specification &state, const int joint)
{
    if (std::fabs(state.qd(joint)) >= joint_velocity_limits_[joint])
    {
        if (PRINT_LOGS_) printf("Joint %d velocity limit reached: %f \n", joint + 1, state.qd(joint));
        return true;        
    }
    
    return false;
}

bool safety_monitor::position_limit_reached(const state_specification &state, const int joint)
{
    if ((state.q(joint) >= joint_position_limits_max_[joint]) || \
        (state.q(joint) <= joint_position_limits_min_[joint]))
    {
        if (PRINT_LOGS_) printf("Joint %d position limit reached: %f \n", joint + 1, state.q(joint));
        return true; 
    }

    return false;
}

bool safety_monitor::reaching_position_limits(const state_specification &state, const int joint)
{

    if ((joint_position_limits_max_[joint] - state.q(joint)) < joint_position_thresholds_[joint])
    {
        if (state.qd(joint) > 0.05)
        {
            printf("Joint %d is too close to the max limit %f %f \n", joint + 1, state.q(joint), state.qd(joint));
            return true;
        } 
    } 

    else if ((joint_position_limits_min_[joint] - state.q(joint)) > -joint_position_thresholds_[joint])
    {
        if (state.qd(joint) < -0.05)
        {
            printf("Joint %d is too close to the min limit %f %f \n", joint + 1, state.q(joint), state.qd(joint));
            return true;
        }
    } 

    return false;
}

int safety_monitor::monitor_future_state(const int desired_control_mode)
{
    switch (desired_control_mode)
    {   
        /*
            Check if the commaned torques are over torque limits.
            Check if the commanded torques will make a joint go over the position limits.
            If all ok: continue with this control mode.
            Else: stop the robot.
        */
        case control_mode::TORQUE:
            for (int i = 0; i < NUM_OF_JOINTS_; i++)
            {
                if (torque_limit_reached(current_state_, i) || \
                    position_limit_reached(predicted_states_[0], i) || \
                    position_limit_reached(predicted_states_[1], i))
                {
                    if (PRINT_LOGS_) printf("Torque commands not safe \n");
                    return control_mode::STOP_MOTION;
                }
            }

            return control_mode::TORQUE;

        /*
            Check if the commaned velocities are over velocity limits.
            Check if the commanded velocities will make a joint go over position limits.
            If all ok: continue with this control mode.
            Else: stop the robot.
        */
        case control_mode::VELOCITY:
            for (int i = 0; i < NUM_OF_JOINTS_; i++)
            {
                if (velocity_limit_reached(predicted_states_[0], i) || \
                    position_limit_reached(predicted_states_[0], i) || \
                    position_limit_reached(predicted_states_[1], i))
                {
                    if (PRINT_LOGS_) printf("Velocity commands not safe \n");
                    return control_mode::STOP_MOTION;
                }
            }

            return control_mode::VELOCITY;

        /*
            Simple check if the computed postion commands are valid, i.e. over the limit.
            If a position limit reached, stop the program.
            Last step in safety check.
        */
        case control_mode::POSITION:
            for (int i = 0; i < NUM_OF_JOINTS_; i++)
            {
                if (position_limit_reached(predicted_states_[0], i))
                {
                    if (PRINT_LOGS_) printf("Position commands not safe \n");
                    return control_mode::STOP_MOTION;
                }
            }

            return control_mode::POSITION;

        default: return control_mode::STOP_MOTION;
    }
}