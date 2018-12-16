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

#include <youbot_mediator.hpp>
#include <model_prediction.hpp>
#include <state_specification.hpp>
#include <command_specification.hpp>
#include <task_specification.hpp>
#include <iostream>
#include <sstream>
#include <fstream>
#include <time.h>
#include <cmath>
#include <boost/assign/list_of.hpp>
#include <stdlib.h> /* abs */
#include <unistd.h>

const int JOINTS = 5;
const int NUMBER_OF_CONSTRAINTS = 6;
const int MILLISECOND = 1000;

void initialize_variables(const KDL::Chain &arm_chain_,
                 state_specification &motion_,
                 command_specification &commands,
                 const int NUMBER_OF_CONSTRAINTS)
{

    int number_of_segments = arm_chain_.getNrOfSegments();
    int number_of_joints = arm_chain_.getNrOfJoints();

    motion_.init_state(number_of_joints,
                       number_of_segments,
                       number_of_segments + 1,
                       NUMBER_OF_CONSTRAINTS);
    commands.init_commands(number_of_joints);

    // external forces on the arm
    KDL::Wrench externalForce(
        KDL::Vector(0.0, 0.0, 0.0),  //Linear Force
        KDL::Vector(0.0, 0.0, 0.0)); //Torque
    for (int i = 0; i < number_of_segments; i++)
        motion_.external_force[i] = externalForce;

    for (int i = 0; i < number_of_joints; i++){
        motion_.q(i) = 0.0;
        motion_.qd(i) = 0.0;
        motion_.qdd(i) = 0.0;
        motion_.feedforward_torque(i) = 0.0;
        
        commands.q_setpoint(i) = 0.0;
        commands.qd_setpoint(i) = 0.0;
        commands.tau_setpoint(i) = 0.0;
    }

    //Acceleration Constraints on the End-Effector
    KDL::Twist unit_constraint_force(
        KDL::Vector(0.0, 0.0, 0.0),  // linear
        KDL::Vector(0.0, 0.0, 0.0)); // angular

    for(int i = 0; i < NUMBER_OF_CONSTRAINTS; i++)
    {
        motion_.ee_unit_constraint_forces.setColumn(i, unit_constraint_force);
        motion_.ee_acceleration_energy(i) = 0.0;
    }
}

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
    // double navigation[] = {2.9496, 1.0, -1.53240, 2.85214, 2.93816};
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

//Set velocities of arm's joints to 0 value
void stop_motion(youbot_mediator &arm, state_specification &motion){ 
    for (int i = 0; i < JOINTS; i++) motion.qd(i) = 0;  
    arm.set_joint_velocities(motion.qd);
    usleep(5000 * MILLISECOND);
}

void set_ee_constraints(state_specification &motion)
{
    KDL::Twist unit_constraint_force_xl(
        KDL::Vector(0.0, 0.0, 0.0),  // linear
        KDL::Vector(0.0, 0.0, 0.0)); // angular
    motion.ee_unit_constraint_forces.setColumn(0, unit_constraint_force_xl);
    motion.ee_acceleration_energy(0) = 0.000001;

    KDL::Twist unit_constraint_force_yl(
        KDL::Vector(0.0, 0.0, 0.0),  // linear
        KDL::Vector(0.0, 0.0, 0.0)); // angular
    motion.ee_unit_constraint_forces.setColumn(1, unit_constraint_force_yl);
    motion.ee_acceleration_energy(1) = 0.0;

    KDL::Twist unit_constraint_force_zl(
        KDL::Vector(0.0, 0.0, 1.0),  // linear
        KDL::Vector(0.0, 0.0, 0.0)); // angular
    motion.ee_unit_constraint_forces.setColumn(2, unit_constraint_force_zl);
    motion.ee_acceleration_energy(2) = 0.000001;

    KDL::Twist unit_constraint_force_xa(
        KDL::Vector(0.0, 0.0, 0.0),  // linear
        KDL::Vector(0.0, 0.0, 0.0)); // angular
    motion.ee_unit_constraint_forces.setColumn(3, unit_constraint_force_xa);
    motion.ee_acceleration_energy(3) = 0.0;

    KDL::Twist unit_constraint_force_ya(
        KDL::Vector(0.0, 0.0, 0.0),  // linear
        KDL::Vector(0.0, 0.0, 0.0)); // angular
    motion.ee_unit_constraint_forces.setColumn(4, unit_constraint_force_ya);
    motion.ee_acceleration_energy(4) = 0.0;

    KDL::Twist unit_constraint_force_za(
        KDL::Vector(0.0, 0.0, 0.0),  // linear
        KDL::Vector(0.0, 0.0, 0.0)); // angular
    motion.ee_unit_constraint_forces.setColumn(5, unit_constraint_force_za);
    motion.ee_acceleration_energy(5) = 0.0;
}

void set_ext_forces(state_specification &forces){
    forces.external_force[forces.external_force.size() - 1] = \
                                    KDL::Wrench (KDL::Vector(0.0,
                                                            0.0,
                                                            0.0), //Linear Force
                                                KDL::Vector(0.0,
                                                            0.0,
                                                            0.0)); //Torque
}

int main(int argc, char **argv)
{
    double joint_velocity_limits[JOINTS] = {1.5707, 0.8, 1.0, 1.5707, 1.5707};
    
    KDL::Chain arm_chain_;
    state_specification motion_;
    command_specification commands_;
    youbot_mediator arm("/home/djole/Master/Thesis/GIT/MT_testing/youbot_driver/config");
    
    if (arm.initialize(arm_chain_, "arm_link_0", "arm_link_5",
        "/home/djole/Master/Thesis/GIT/MT_testing/Controller/urdf/youbot_arm_only.urdf")\
        != 0) {
            std::cout << "Robot not initialized!" << std::endl;
            return 0;
        }

    model_prediction predictor(arm_chain_);
    int number_of_segments = arm_chain_.getNrOfSegments();
    int number_of_joints = arm_chain_.getNrOfJoints();
    assert(JOINTS == number_of_segments);

    std::vector<KDL::Twist> frame_acceleration_;
    frame_acceleration_.resize(number_of_segments + 1);
    initialize_variables(arm_chain_, motion_, commands_, NUMBER_OF_CONSTRAINTS);
    stop_motion(arm, motion_);

    // go_candle_1(arm);
    // arm.get_joint_positions(motion_.q);
    // arm.get_joint_velocities(motion_.qd);

    // std::cout << "\n" << "Joint Positions" << motion_.q << std::endl;
    // std::cout << "\n" <<"Joint Velocities"<< motion_.qd << std::endl;
    // return 0;
    
    //Create End_effector Cartesian Acceleration task 
    // set_ee_constraints(motion_);
    
    //Create External Forces task 
    // set_ext_forces(motion_);

    //arm root acceleration
    KDL::Vector linearAcc(0.0, 0.0, -9.81); //gravitational acceleration along Z
    KDL::Vector angularAcc(0.0, 0.0, 0.0);
    KDL::Twist root_acc(linearAcc, angularAcc);

    KDL::Solver_Vereshchagin hd_solver_(arm_chain_, root_acc,
                                        NUMBER_OF_CONSTRAINTS);

    //loop with Hz
    double rate = MILLISECOND;

    //Time sampling interval
    double dt = 1.0 / rate;
    // std::cout << "dt: " << dt << "\n";

    // usleep(5000 * MILLISECOND);
    std::cout << "Contro Loop Starts"<< std::endl;
    while (1)
    {
        arm.get_joint_positions(motion_.q);
        arm.get_joint_velocities(motion_.qd);
        // std::cout << "Joints  Pos: " << motion_.q << '\n';
        // std::cout << "Joints  Vel: " << motion_.qd << '\n';
        // std::cout << "Joints  Acc: " << motion_.qdd << '\n' << "\n";
        // for (int i=0; i< number_of_segments; i++)
        //     std::cout<<"FORCES \n"<<motion_.external_force[i]<<std::endl;

        int result = hd_solver_.CartToJnt(motion_.q,
                                          motion_.qd,
                                          motion_.qdd,
                                          motion_.ee_unit_constraint_forces,
                                          motion_.ee_acceleration_energy,
                                          motion_.external_force,
                                          motion_.feedforward_torque);

        // std::cout << "Solver return: " << result << '\n';
        // std::cout << "Joints  Acc: " << motion_.qdd << '\n' << "\n";
        assert(result == 0);
        predictor.integrate(motion_, commands_, dt);

        // std::cout << "\n Joint Vel::Commanded: ";
        // for (int i = 0; i < JOINTS; i++)
            // std::cout << qd_setpoint[i].angularVelocity.value() << " , ";
        // std::cout << "\n";

        // hd_solver_.get_transformed_link_acceleration(frame_acceleration_);
        // std::cout << "\n \n Frame ACC" << '\n';
        // for (size_t i = 0; i < arm_chain_.getNrOfSegments() + 1; i++)
        //     std::cout << frame_acceleration_[i] << '\n';
        // std::cout << std::endl;

        // KDL::JntArray control_torque_Ver(number_of_joints);
        // hd_solver_.get_control_torque(control_torque_Ver);
        // std::cout << "\n" << "Joint torques: " << control_torque_Ver << '\n';
        // std::cout << "\n";

        arm.set_joint_velocities(commands_.qd_setpoint);
        usleep(MILLISECOND);
    }
    return 0;
}
