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

#ifndef YOUBOT_CONSTANTS_HPP
#define YOUBOT_CONSTANTS_HPP

#include <stdlib.h> /* abs */
#include <unistd.h>
#include <cmath>

#define PI 3.14159265358979323846
#define DEG_TO_RAD(x) (x) * PI / 180.0

namespace youbot_constants
{
    // Number of joints in the manipulator
    const int NUMBER_OF_JOINTS(5);
    const int NUMBER_OF_SEGMENTS(5);
    const int NUMBER_OF_FRAMES(6);

    //Arm's root acceleration
    const std::vector<double> root_acceleration {0.0, 0.0, 9.81, 0.0, 0.0, 0.0};

    //Kuka youBot store position limit values: positive and negative
    const std::vector<double> joint_position_limits_max_1 {  2.9496,  1.5707,  2.5481,  1.7889,  2.9234};
    const std::vector<double> joint_position_limits_min_1 { -2.9496, -1.1344, -2.6354, -1.7889, -2.9234};

    //Position limit values from URDF file: positive and negative
    const std::vector<double> joint_position_limits_max_2 {5.89921, 2.70526,  0.00000, 3.57792, 5.84685};
    const std::vector<double> joint_position_limits_min_2 {0.00000, 0.00000, -5.16617, 0.00000, 0.00000};
    // const std::vector<double> joint_position_limits_max_2 { 5.899210,  2.705260,  0.000001,  3.577920,  5.846850};
    // const std::vector<double> joint_position_limits_min_2 {-0.000001, -0.000001, -5.166170, -0.000001, -0.000001};
    
    const std::vector<double> joint_position_limits_max_2_sim { 5.899210,  2.705260,  0.000001,  3.577920,  5.846850};
    const std::vector<double> joint_position_limits_min_2_sim {-0.000001, -0.000001, -5.166170, -0.000001, -0.000001};

    const std::vector<double> joint_position_thresholds {DEG_TO_RAD(10), DEG_TO_RAD(10), DEG_TO_RAD(10), DEG_TO_RAD(8), DEG_TO_RAD(5)};

    // Robocup URDF file parameters for velocity limits
    // std::vector<double> joint_velocity_limits {1.5707, 0.8, 1.0, 1.5707, 1.5707};

    // YouBot Store velocity limits
    const std::vector<double> joint_velocity_limits {1.5707, 01.5707, 1.5707, 1.5707, 1.5707};

    // JP's max torques
    // std::vector<double> joint_torque_limits {12.9012, 12.9012, 8.2700, 4.1748, 1.7550};

    // youBot store's max torques 
    const std::vector<double> joint_torque_limits {9.5, 9.5, 6.0, 2.0, 1.0};

    // Benjamin Keiser' max torques (fast version)
    // std::vector<double> joint_torque_limits {17.0, 17.0, 8.0, 2.7, 1.0}};

    // Offsets required for the youBot store model: Negative Candle config values -Robocup
    // std::vector<double> youbot_joint_offsets {-2.1642, -1.13446, 2.54818, -1.78896, -2.9234};

    // Offsets required for the youBot store model: Negative Candle config values - JP 
    const std::vector<double> joint_offsets {-2.94960, -1.13446, 2.54818, -1.78896, -2.9234};

    // Offsets required for the youBot store model: Negative Candle config values -Sven's
    // std::vector<double> joint_offsets {-2.9496, -1.1344, 2.6354, -1.7890, -2.9234};

    // Offsets required for the youBot store model: Negative Candle config values: keiser's
    // std::vector<double> joint_offsets {-2.9496, -1.1344, 2.5481, -1.7889, -2.9234};

    // Rotor inertia - "d" in the algorithm: Computed from youBot store values
    const std::vector<double> joint_inertia {0.33848, 0.33848, 0.13571, 0.04698, 0.01799};

    const std::string config_path = "/home/djole/Master/Thesis/GIT/MT_testing/youbot_driver/config";
    
    // const std::string urdf_path = "/home/djole/Master/Thesis/GIT/MT_testing/Controller/urdf/youbot_arm_only.urdf";
    const std::string urdf_path = "/home/djole/Master/Thesis/GIT/MT_testing/Controller/urdf/youbot_arm_zero_inertia.urdf";
    // const std::string urdf_path = "/home/djole/Master/Thesis/GIT/MT_testing/Controller/urdf/youbot_arm_zero_inertia_new_mass.urdf";

    const std::string root_name = "arm_link_0";
    const std::string tooltip_name = "arm_link_5";
}

#endif /* YOUBOT_CONSTANTS_HPP */