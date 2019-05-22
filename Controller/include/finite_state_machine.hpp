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

#ifndef FINITE_STATE_MACHINE_HPP
#define FINITE_STATE_MACHINE_HPP
#include <state_specification.hpp>
#include <constants.hpp>
#include <fk_vereshchagin.hpp>
#include <kdl_eigen_conversions.hpp>
#include <motion_profile.hpp>
#include <iostream>
#include <sstream>
#include <fstream>
#include <unistd.h>
#include <cmath>
#include <stdlib.h>     /* abs */

enum task_model
{
  full_pose = 0,
  moveGuarded = 1,
  moveTo = 2
};

enum control_status
{
    STOP_CONTROL = -1,
    NOMINAL = 0,
    START_TO_CRUISE = 1,
    CRUISE_TO_STOP = 2,
    CRUISE_THROUGH_TUBE = 3,
    CRUISE = 4,
    STOP_ROBOT = 5
};

struct moveTo_task
{
    KDL::Frame tf_pose, goal_pose;
    std::vector<double> tube_start_position{std::vector<double>(3, 0.0)};
    std::vector<double> tube_tolerances{std::vector<double>(6, 0.0)};
    double tube_speed = 0.0;
    double contact_threshold_linear = 0.0;
    double contact_threshold_angular = 0.0;
    double time_limit = 0.0;
};

struct full_pose_task
{
    KDL::Frame tf_pose, goal_pose;
    double contact_threshold_linear = 0.0;
    double contact_threshold_angular = 0.0;
    double time_limit = 0.0;
};

class finite_state_machine
{
    public:
        finite_state_machine(const int num_of_joints,
                             const int num_of_segments,
                             const int num_of_frames,
                             const int num_of_constraints);
        ~finite_state_machine(){};

        int initialize_with_moveTo(const moveTo_task &task);
        int initialize_with_full_pose(const full_pose_task &task);

        int update(const state_specification &robot_state,
                   state_specification &desired_state,
                   const double time_passed_sec);

    private:
        const int NUM_OF_JOINTS_, NUM_OF_SEGMENTS_, NUM_OF_FRAMES_, NUM_OF_CONSTRAINTS_;
        const int END_EFF_;
        int desired_task_model_;
        double total_control_time_sec_;
        bool goal_reached_, time_limit_reached_, contact_detected_;
        state_specification robot_state_, desired_state_;
        moveTo_task moveTo_task_;
        full_pose_task full_pose_task_;

        int update_full_pose_task(state_specification &desired_state);
        int update_moveTo_task(state_specification &desired_state);
};
#endif /* FINITE_STATE_MACHINE_HPP */