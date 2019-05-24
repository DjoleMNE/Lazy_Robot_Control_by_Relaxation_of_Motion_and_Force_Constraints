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

#include <finite_state_machine.hpp>

finite_state_machine::finite_state_machine(const int num_of_joints,
                                           const int num_of_segments,
                                           const int num_of_frames,
                                           const int num_of_constraints):
    NUM_OF_JOINTS_(num_of_joints), NUM_OF_SEGMENTS_(num_of_segments),
    NUM_OF_FRAMES_(num_of_frames), NUM_OF_CONSTRAINTS_(num_of_constraints),
    END_EFF_(NUM_OF_SEGMENTS_ - 1), desired_task_model_(task_model::full_pose),
    motion_profile_(m_profile::CONSTANT), total_control_time_sec_(0.0),
    goal_reached_(false), time_limit_reached_(false), contact_detected_(false),
    robot_state_(NUM_OF_JOINTS_, NUM_OF_SEGMENTS_, NUM_OF_FRAMES_, NUM_OF_CONSTRAINTS_),
    desired_state_(robot_state_)
{
}

int finite_state_machine::initialize_with_moveTo(const moveTo_task &task,
                                                 const int motion_profile)
{
    desired_task_model_ = task_model::moveTo;
    moveTo_task_ = task;
    motion_profile_ = motion_profile;

    return control_status::NOMINAL;
}

int finite_state_machine::initialize_with_full_pose(const full_pose_task &task,
                                                    const int motion_profile)
{
    desired_task_model_ = task_model::full_pose;
    full_pose_task_ = task;
    motion_profile_ = motion_profile;

    return control_status::NOMINAL;
}

int finite_state_machine::update_moveTo_task(state_specification &desired_state)
{
    if(total_control_time_sec_ > moveTo_task_.time_limit) 
    {
        desired_state.frame_velocity[END_EFF_].vel(0) = 0.0;
        #ifndef NDEBUG       
            if(!time_limit_reached_) printf("Time limit reached\n");
            time_limit_reached_ = true;
        #endif
        return control_status::STOP_ROBOT;
    }

    double x_error = desired_state_.frame_pose[END_EFF_].p(0) -\
                     robot_state_.frame_pose[END_EFF_].p(0);

    /*
     * See if the robot has reached goal area in X linear direction.
     * If yes command zero speed, to keep it in that area.
     * Else go with initially commanded speed tube.
    */
    if ( std::fabs(x_error) <= moveTo_task_.tube_tolerances[0] )
    {
        desired_state.frame_velocity[END_EFF_].vel(0) = 0.0;
        #ifndef NDEBUG       
            if(!goal_reached_) printf("Goal area reached\n");
            goal_reached_ = true;
        #endif
        return control_status::STOP_ROBOT;
    }
    
    else
    {
        double speed;
        switch (motion_profile_)
        {
            case m_profile::STEP:
                speed = motion_profile::negative_step_function(std::fabs(x_error), 
                                                               moveTo_task_.tube_speed, 
                                                               0.25, 0.4, 0.2);
                break;

            case m_profile::S_CURVE:
                speed = motion_profile::s_curve_function(std::fabs(x_error), 
                                                         0.05, moveTo_task_.tube_speed, 5.0);
                break;

            default:
                speed = moveTo_task_.tube_speed;
                break;
        }

        if (sign(x_error) == -1) desired_state.frame_velocity[END_EFF_].vel(0) = -1 * speed;      
        else desired_state.frame_velocity[END_EFF_].vel(0) = speed;              
    }
     
    return control_status::NOMINAL;
}

int finite_state_machine::update_full_pose_task(state_specification &desired_state)
{
    if(total_control_time_sec_ > full_pose_task_.time_limit) 
    {
        desired_state.frame_pose[END_EFF_] = robot_state_.frame_pose[END_EFF_];
        #ifndef NDEBUG
            if(!time_limit_reached_) printf("Time limit reached\n");
            time_limit_reached_ = true;
        #endif

        return control_status::STOP_ROBOT;
    }

    return control_status::NOMINAL;
}

int finite_state_machine::update(const state_specification &robot_state,
                                 state_specification &desired_state,
                                 const double time_passed_sec)
{
    robot_state_            = robot_state;
    desired_state_          = desired_state;
    total_control_time_sec_ = time_passed_sec;

    switch (desired_task_model_)
    {
        case task_model::moveTo:
            return update_moveTo_task(desired_state);
            break;
        
        case task_model::full_pose:
            return update_full_pose_task(desired_state);
            break;
            
        default:
            printf("Unsupported task model\n");
            return control_status::STOP_CONTROL;
            break;
    }
}


int finite_state_machine::sign(double x)
{
    if (x > 0.0) return 1;
    else if (x < 0.0) return -1;
    else return 0;
}