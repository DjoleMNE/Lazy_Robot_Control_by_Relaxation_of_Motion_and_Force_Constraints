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
#include <kdl_parser/kdl_parser.hpp>
#include <youbot_custom_model.hpp>
#include <urdf/model.h>
#include <youbot_mediator.hpp>
#include <state_specification.hpp>
#include <dynamics_controller.hpp>
#include <iostream>
#include <sstream>
#include <fstream>
#include <chrono>
#include <thread>         // std::this_thread::sleep_for
#include <time.h>
#include <cmath>
#include <boost/assign/list_of.hpp>
#include <stdlib.h> /* abs */
#include <unistd.h>

const int JOINTS = 5;
const int NUMBER_OF_CONSTRAINTS = 6;
const int MILLISECOND = 1000;
const long SECOND = 1000000;

// Go to Candle 1 configuration  
void go_candle_1(youbot_mediator &arm){
    KDL::JntArray candle_pose(JOINTS);
    double candle[] = {2.1642, 1.13446, -2.54818, 1.78896, 0.12};
    for (int i = 0; i < JOINTS; i++) candle_pose(i) = candle[i];  
    arm.set_joint_positions(candle_pose);
    usleep(5000 * MILLISECOND);
}

// Go to Candle 2 configuration  
void go_candle_2(youbot_mediator &arm){
    KDL::JntArray candle_pose(JOINTS);
    double candle[] = {2.9496, 1.1344, -2.6354, 1.7890, 2.9234};
    for (int i = 0; i < JOINTS; i++) candle_pose(i) = candle[i];  
    arm.set_joint_positions(candle_pose);
    usleep(5000 * MILLISECOND);
}

// Go to Candle 3 configuration  
void go_candle_3(youbot_mediator &arm){
    KDL::JntArray candle_pose(JOINTS);
    double candle[] = {2.9496, 1.1344, -2.54818, 1.78896, 2.9234};
    for (int i = 0; i < JOINTS; i++) candle_pose(i) = candle[i];  
    arm.set_joint_positions(candle_pose);
    usleep(5000 * MILLISECOND);
}

// Go to Folded configuration  
void go_folded(youbot_mediator &arm){
    KDL::JntArray folded_pose(JOINTS);
    double folded[] = {0.02, 0.02, -0.02, 0.023, 0.12};
    for (int i = 0; i < JOINTS; i++) folded_pose(i) = folded[i];  
    arm.set_joint_positions(folded_pose);
    usleep(5000 * MILLISECOND);
}

// Go to Navigation 1 configuration  
void go_navigation_1(youbot_mediator &arm){
    KDL::JntArray desired_pose(JOINTS);
    double navigation[] = {2.9496, 0.075952, -1.53240, 3.35214, 2.93816};
    for (int i = 0; i < JOINTS; i++) desired_pose(i) = navigation[i];  
    arm.set_joint_positions(desired_pose);
    usleep(5000 * MILLISECOND);
}

// Go to Navigation 2 configuration  
void go_navigation_2(youbot_mediator &arm){
    KDL::JntArray desired_pose(JOINTS);
    double navigation[] = {2.9496, 1.0, -1.53240, 2.85214, 2.93816};
    for (int i = 0; i < JOINTS; i++) desired_pose(i) = navigation[i];  
    arm.set_joint_positions(desired_pose);
    usleep(5000 * MILLISECOND);
}

// Go to Navigation 3 configuration  
void go_navigation_3(youbot_mediator &arm){
    KDL::JntArray desired_pose(JOINTS);
    double navigation[] = {1.3796, 1.0, -1.53240, 2.85214, 2.93816};
    for (int i = 0; i < JOINTS; i++) desired_pose(i) = navigation[i];  
    arm.set_joint_positions(desired_pose);
    usleep(5000 * MILLISECOND);
}

//Set velocities of arm's joints to 0 value
void stop_robot_motion(youbot_mediator &arm, state_specification &motion){ 
    for (int i = 0; i < JOINTS; i++) motion.qd(i) = 0;  
    arm.set_joint_velocities(motion.qd);
}

int main(int argc, char **argv)
{
    youbot_mediator robot_driver;

    bool simulation_environment = true;
    bool use_custom_model = false;

    const std::string config_path = "/home/djole/Master/Thesis/GIT/MT_testing/youbot_driver/config";
    
    // const std::string urdf_path = "/home/djole/Master/Thesis/GIT/MT_testing/Controller/urdf/youbot_arm_only.urdf";
    const std::string urdf_path = "/home/djole/Master/Thesis/GIT/MT_testing/Controller/urdf/youbot_arm_zero_inertia.urdf";

    const std::string root_name = "arm_link_0";
    const std::string tooltip_name = "arm_link_5";
    
    // Extract robot model and if not simulation, establish connection with motor drivers
    robot_driver.initialize(config_path, root_name, tooltip_name, urdf_path, 
                            use_custom_model, simulation_environment);
    
    int number_of_segments = robot_driver.get_robot_model().getNrOfSegments();
    int number_of_joints = robot_driver.get_robot_model().getNrOfJoints();

    assert(JOINTS == number_of_segments);

    state_specification motion_(number_of_joints,
                                number_of_segments,
                                number_of_segments + 1,
                                NUMBER_OF_CONSTRAINTS);
    state_specification commands_(motion_);

    if(!simulation_environment){
        assert(("Robot is not initialized", robot_driver.is_initialized));
        stop_robot_motion(robot_driver, motion_);
        // go_navigation_2(robot_driver);
        go_folded(robot_driver);
        // go_candle_3(robot_driver);
        // robot_driver.get_joint_positions(motion_.q);
        // robot_driver.get_joint_velocities(motion_.qd);
        // return 0;
    }

    //loop rate in Hz
    int rate_hz = 999;
    dynamics_controller controller(robot_driver, rate_hz);
    
    //Create End_effector Cartesian Acceleration task 
    controller.define_ee_constraint_task(std::vector<bool>{false, true, true, 
                                                           false, false, false},
                                         std::vector<double>{1.0, 5.0, 15.0, 
                                                             0.0, 0.0, 0.0});
    //Create External Forces task 
    controller.define_ee_external_force_task(std::vector<double>{0.0, 0.0, 0.0, 
                                                                 0.0, 0.0, 0.0});
    //Create Feedforward torques task 
    controller.define_feadforward_torque_task(std::vector<double>{0.0, 0.0, 
                                                                  0.0, 0.0, 
                                                                  0.0});    
    controller.control(simulation_environment, 
                       control_mode::velocity_control);

    return 0;
}