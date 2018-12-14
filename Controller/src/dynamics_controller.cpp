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

#include <dynamics_controller.hpp>
#include <iostream>
#include <sstream>
#include <time.h>
#include <boost/assign/list_of.hpp>
#include <fstream>
#include <unistd.h>
#include <cmath>
#include <stdlib.h>     /* abs */

dynamics_controller::dynamics_controller(
    const KDL::Chain &chain,
    const KDL::Twist &root_acc,
    std::vector<double> &joint_position_limits,
    std::vector<double> &joint_velocity_limits,
    std::vector<double> &joint_acceleration_limits,
    std::vector<double> &joint_torque_limits,
    double rate_hz):
        chain_(chain),
        hd_solver_(chain_, root_acc, NUMBER_OF_CONSTRAINTS),
        joint_position_limits_(joint_position_limits),
        joint_velocity_limits_(joint_velocity_limits),
        joint_acceleration_limits_(joint_acceleration_limits),
        joint_torque_limits_(joint_torque_limits),
        rate_hz_(rate_hz)
{
    number_of_frames_ = chain_.getNrOfSegments() + 1;
    number_of_joints_ = chain_.getNrOfJoints();
    number_of_segments_ = chain_.getNrOfSegments();
    assert(number_of_joints_ == number_of_segments_);
     
    current_state_.init_state(number_of_joints_, 
                    number_of_segments_, 
                    number_of_frames_, 
                    NUMBER_OF_CONSTRAINTS);
    desired_state_ = current_state_;
    predicted_state_ = current_state_;

    reset_state(desired_state_);
    reset_state(current_state_);
    reset_state(predicted_state_);

    dt_ = 1.0 / rate_hz_;
}

void dynamics_controller::check_limits(state_specification state)
{
    //If joint limit reached, stop the program
    for (int i = 0; i < number_of_joints_; i++)
    {
        if (abs(state.qd(i)) > joint_velocity_limits_[i])
        {
            std::cout << "Joint "<< i << "rate: " 
                      <<state.qd(i) << " rad/s over limit" 
                      << std::endl;
        }
        assert(abs(state.qd(i)) <= joint_velocity_limits_[i]);
    }
}

// Set all values of desired state to 0 
void dynamics_controller::reset_desired_state(){
    reset_state(desired_state_);
}

// Set all values of a state to 0 
void dynamics_controller::reset_state(state_specification state)
{
    //Joint state
    for (int i = 0; i < number_of_joints_; i++){
        state.q(i) = 0.0;
        state.qd(i) = 0.0;
        state.qdd(i) = 0.0;
        state.feedforward_torque(i) = 0.0;
        state.control_torque(i) = 0.0;
    }

    //Cartesian state - Frames
    for (int i = 0; i < number_of_frames_; i++)
    {
        // KDL::SetToZero(state.frame_pose[i]);
        // KDL::SetToZero(state.frame_velocity[i]);
        KDL::SetToZero(state.frame_acceleration[i]);
    }

    //External forces on the arm's segments
    KDL::Wrench externalForce(
        KDL::Vector(0.0, 0.0, 0.0),  //Linear Force
        KDL::Vector(0.0, 0.0, 0.0)); //Torque

    for (int i = 0; i < number_of_segments_; i++){
        state.external_force[i] = externalForce;
    }

    //Acceleration Constraints on the End-Effector
    KDL::Twist unit_constraint_force(
        KDL::Vector(0.0, 0.0, 0.0),  // linear
        KDL::Vector(0.0, 0.0, 0.0)); // angular

    for(int i = 0; i < NUMBER_OF_CONSTRAINTS; i++)
    {
        state.ee_unit_constraint_forces.setColumn(0, unit_constraint_force);
        state.ee_acceleration_energy(0) = 0.0;
    }
}

// Define Cartesian Acceleration task on end-effector
void dynamics_controller::set_ee_constraints(
                        std::vector<bool> &constraint_direction, 
                        std::vector<double> &cartesian_acceleration,
                        state_specification &state)
{    
    // Set directions in which constraint force should work 
    KDL::Twist unit_force_x_l(
        KDL::Vector((constraint_direction[0] ? 1.0 : 0.0), 0.0, 0.0), 
        KDL::Vector(0.0, 0.0, 0.0));
    state.ee_unit_constraint_forces.setColumn(0, unit_force_x_l);

    KDL::Twist unit_force_y_l(
            KDL::Vector(0.0, (constraint_direction[1] ? 1.0 : 0.0), 0.0),
            KDL::Vector(0.0, 0.0, 0.0));
    state.ee_unit_constraint_forces.setColumn(1, unit_force_y_l);

    KDL::Twist unit_force_z_l(
            KDL::Vector(0.0, 0.0, (constraint_direction[2] ? 1.0 : 0.0)),
            KDL::Vector(0.0, 0.0, 0.0));
    state.ee_unit_constraint_forces.setColumn(2, unit_force_z_l);

    KDL::Twist unit_force_x_a(
            KDL::Vector(0.0, 0.0, 0.0),
            KDL::Vector((constraint_direction[3] ? 1.0 : 0.0), 0.0, 0.0));
    state.ee_unit_constraint_forces.setColumn(3, unit_force_x_a);

    KDL::Twist unit_force_y_a(
            KDL::Vector(0.0, 0.0, 0.0),
            KDL::Vector(0.0, (constraint_direction[4] ? 1.0 : 0.0), 0.0));
    state.ee_unit_constraint_forces.setColumn(4, unit_force_y_a);

    KDL::Twist unit_force_z_a(
            KDL::Vector(0.0, 0.0, 0.0),
            KDL::Vector(0.0, 0.0, (constraint_direction[5] ? 1.0 : 0.0)));
    state.ee_unit_constraint_forces.setColumn(5, unit_force_z_a);

    // Set desired acceleration on the end-effector
    for (int i = 0; i<NUMBER_OF_CONSTRAINTS; i++)
        state.ee_acceleration_energy(i) = cartesian_acceleration[i];
}

void dynamics_controller::integrate_robot_motion(
    state_specification current_state,
    state_specification predicted_state,
    int number_of_steps)
{
    double step_size = dt_ * number_of_steps; 

    //Euler method
    for (int i = 0; i < number_of_joints_; i++)
    {
        /* std::cout << joint_state[i].angularVelocity.value() 
                     << " " << abs(joint_command.data[i]) << std::endl; */
        predicted_state.qd(i) = current_state.qd(i)\
                                + current_state.qdd(i) * step_size;
        predicted_state.q(i) = current_state.q(i)\
                            + (predicted_state.qd(i)\
                            - current_state.qdd(i) * step_size / 2) * step_size;
    }
}

int dynamics_controller::control(youbot::YouBotManipulator &robot)
{
    // std::cout << "dt: " << dt << "\n";
    // usleep(5000 * MILLISECOND);

    // while (1)
    // {
    //     robot.getJointData(q);
    //     robot.getJointData(qd);

    //     for (int i = 0; i < number_of_joints_; i++)
    //     {
    //         current_state_.q(i) = q[i].angle.value();
    //         current_state_.qd(i) = qd[i].angularVelocity.value();
    //         current_state_.qdd(i) = 0.0;
    //     }

    //     // std::cout << "Joints  Pos: " << current_state_.q << '\n';
    //     // std::cout << "Joints  Vel: " << current_state_.qd << '\n';

    //     int result = hd_solver_.CartToJnt(current_state_.q,
    //                                       current_state_.qd,
    //                                       current_state_.qdd,                                       //qdd_ is overwritten by resulting acceleration
    //                                       current_state_.ee_unit_constraint_forces,       // alpha
    //                                       current_state_.ee_acceleration_energy, // beta
    //                                       current_state_.external_force,
    //                                       current_state_.feedforward_torque);

    //     // std::cout << "Solver return: " << result << '\n';
    //     assert(result == 0);

    //     // std::cout << "Joints  Acc: " << current_state_.qdd << '\n' << "\n";
    //     integrate_robot_motion(current_state_.q, 
    //                            current_state_.qd, 
    //                            current_state_.qdd, 
    //                            command_.q, command_.qd, joint_limit, 1);

    //     // std::cout << "\n Joint Vel::Commanded: ";
    //     // for (int i = 0; i < JOINTS; i++)
    //     // {
    //     //     std::cout << qd_setpoint[i].angularVelocity.value() << " , ";
    //     // }
    //     // std::cout << "\n";

    //     // hd_solver_.get_transformed_link_acceleration(frame_acceleration_);

    //     // std::cout << "\n \n Frame ACC" << '\n';
    //     // for (size_t i = 0; i < arm_chain_.getNrOfSegments() + 1; i++)
    //     // {
    //     //     std::cout << frame_acceleration_[i] << '\n';
    //     // }

    //     // KDL::JntArray control_torque_Ver(number_of_joints);
    //     // hd_solver_.get_control_torque(control_torque_Ver);
    //     // std::cout << "\n"
    //     //           << "Joint torques:        " << control_torque_Ver << '\n';

    //     // std::cout << "\n";

    //     arm.setJointData(qd_setpoint);

    //     // stopMotion();

    //     usleep(MILLISECOND);
    // }

    // //Linux sleep
    // // usleep(5000 * MILLISECOND);

    // // for (int i = 0; i < JOINTS; i++)
    // // {
    // //     qd_setpoint[i].angularVelocity = 0.0 * radian_per_second;
    // // }

    // // arm.setJointData(qd_setpoint);

    return 0;
                    
}
