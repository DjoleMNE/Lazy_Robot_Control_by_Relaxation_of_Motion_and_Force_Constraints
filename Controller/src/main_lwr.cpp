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
#include <state_specification.hpp>
#include <dynamics_controller.hpp>
#include <iostream>
#include <utility> 
#include <sstream>
#include <fstream>
#include <chrono>
#include <thread>         // std::this_thread::sleep_for
#include <time.h>
#include <cmath>
#include <boost/assign/list_of.hpp>
#include <stdlib.h> /* abs */
#include <unistd.h>

const int JOINTS = 7;
const int NUMBER_OF_CONSTRAINTS = 6;
const int MILLISECOND = 1000;
const long SECOND = 1000000;

int environment_ = lwr_environment::LWR_SIMULATION;
int robot_model_ = lwr_model::LWR_URDF;

// Go to Candle 1 configuration  
void go_candle(lwr_mediator &arm){
    KDL::JntArray candle_pose(JOINTS);
    double candle[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    for (int i = 0; i < JOINTS; i++) candle_pose(i) = candle[i];  
    arm.set_joint_positions(candle_pose);
    if(environment_ != lwr_environment::LWR_SIMULATION) usleep(5000 * MILLISECOND);
}

// Go to Folded configuration  
void go_folded(lwr_mediator &arm){
    KDL::JntArray folded_pose(JOINTS);
    double folded[] = {-1.1572, -1.9104, -2.5334, -1.7853, -0.0727, 0.9176, -1.88766};
    for (int i = 0; i < JOINTS; i++) folded_pose(i) = folded[i];  
    arm.set_joint_positions(folded_pose);
    if(environment_ != lwr_environment::LWR_SIMULATION) usleep(5000 * MILLISECOND);
}

// Go to Navigation 1 configuration  
void go_navigation(lwr_mediator &arm){
    KDL::JntArray desired_pose(JOINTS);
    double navigation[] = {1.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.0};
    for (int i = 0; i < JOINTS; i++) desired_pose(i) = navigation[i];  
    arm.set_joint_positions(desired_pose);
    if(environment_ != lwr_environment::LWR_SIMULATION) usleep(5000 * MILLISECOND);
}

//Set velocities of arm's joints to 0 value
void stop_robot_motion(lwr_mediator &arm, state_specification &motion){ 
    for (int i = 0; i < JOINTS; i++) motion.qd(i) = 0;  
    arm.set_joint_velocities(motion.qd);
}

int main(int argc, char **argv)
{
    printf("LWR MAIN Started \n");
    lwr_mediator robot_driver;

    environment_ = lwr_environment::LWR_SIMULATION;
    robot_model_ = lwr_model::LWR_URDF;

    // Extract robot model and if not simulation, establish connection with motor drivers
    robot_driver.initialize(robot_model_, environment_);
    
    int number_of_segments = robot_driver.get_robot_model().getNrOfSegments();
    int number_of_joints = robot_driver.get_robot_model().getNrOfJoints();
    
    assert(JOINTS == number_of_segments);

    state_specification motion_(number_of_joints,
                                number_of_segments,
                                number_of_segments + 1,
                                NUMBER_OF_CONSTRAINTS);
                                

    assert(("Robot is not initialized", robot_driver.is_initialized()));
    stop_robot_motion(robot_driver, motion_);
    go_navigation(robot_driver);

    // robot_driver.get_joint_positions(motion_.q);
    // robot_driver.get_joint_velocities(motion_.qd);
    
    //loop rate in Hz
    int rate_hz = 999;
    dynamics_controller controller(&robot_driver, rate_hz);
    
    //Create End_effector Cartesian Acceleration task 
    controller.define_ee_acc_constraint(std::vector<bool>{false, false, false, // Linear
                                                          false, false, false}, // Angular
                                        std::vector<double>{0.0, 0.0, 0.0, // Linear
                                                            0.0, 0.0, 0.0}); // Angular
    //Create External Forces task 
    controller.define_ee_external_force(std::vector<double>{0.0, 0.0, 0.0, // Linear
                                                            0.0, 0.0, 0.0}); // Angular
    //Create Feedforward torques task s
    controller.define_feedforward_torque(std::vector<double>{0.0, 0.0, 
                                                             0.0, 0.0, 
                                                             0.0, 0.0, 0.0}); 

    controller.define_desired_ee_pose(std::vector<bool>{true, true, true, // Linear
                                                        false, false, false}, // Angular
                                      std::vector<double>{0.262105,  0.004157,  0.308883, // Linear: Vector
                                                          0.338541,  0.137563,  0.930842, // Angular: Rotation Matrix
                                                          0.337720, -0.941106,  0.016253,
                                                          0.878257,  0.308861, -0.365061});

    controller.control(control_mode::TORQUE, true);

    return 0;
}