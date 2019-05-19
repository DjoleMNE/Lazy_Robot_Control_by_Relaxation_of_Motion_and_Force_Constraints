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
    desired_task_model_(task_model::full_pose), total_control_time_sec_(0.0),
    goal_reached_(false), time_limit_reached_(false), contact_detected_(false),
    robot_state_(NUM_OF_JOINTS_, NUM_OF_SEGMENTS_, NUM_OF_FRAMES_, NUM_OF_CONSTRAINTS_),
    desired_state_(robot_state_)
{
}

int finite_state_machine::initialize_with_moveTo(const moveTo_task &task)
{
    desired_task_model_ = task_model::moveTo;
    moveTo_task_ = task;

    return control_status::NOMINAL;
}

int finite_state_machine::initialize_with_full_pose(const full_pose_task &task)
{
    desired_task_model_ = task_model::full_pose;
    full_pose_task_ = task;

    return control_status::NOMINAL;
}

int finite_state_machine::update(const state_specification &robot_state,
                                 state_specification &desired_state,
                                 const double time_passed_sec)
{
    robot_state_        = robot_state;
    desired_state_      = desired_state;
    total_control_time_sec_ = time_passed_sec;

    return control_status::NOMINAL;
}