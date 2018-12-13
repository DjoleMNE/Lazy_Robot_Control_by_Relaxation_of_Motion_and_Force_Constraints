#include <kdl_parser/kdl_parser.hpp>
#include <youbot_driver/youbot/YouBotManipulator.hpp>
#include <urdf/model.h>
#include <dynamics_controller.hpp>
#include <iostream>
#include <unistd.h>
#include <cmath>
#include <stdlib.h>     /* abs */

dynamics_controller::dynamics_controller(
    const KDL::Chain &chain,
    const KDL::Twist &acc_root,
    std::vector<double> joint_position_limits,
    std::vector<double> joint_velocity_limits,
    std::vector<double> joint_acceleration_limits,
    std::vector<double> joint_torque_limits,
    double rate_hz)
    : hd_solver_(chain, acc_root, NUMBER_OF_CONSTRAINTS),
      chain_(chain),
      joint_position_limits_(joint_position_limits),
      joint_velocity_limits_(joint_velocity_limits),
      joint_acceleration_limits_(joint_acceleration_limits),
      joint_torque_limits_(joint_torque_limits)
{
    number_of_frames_ = chain.getNrOfSegments() + 1;
    number_of_joints_ = chain_.getNrOfJoints();
    number_of_segments_ = chain.getNrOfSegments();
    assert(number_of_joints_ == number_of_segments_);
    state_.init_state(number_of_joints_, number_of_segments_, number_of_frames_, NUMBER_OF_CONSTRAINTS);
    rate_hz_ = rate_hz;
    dt_ = 1.0 / rate_hz_;
}

void dynamics_controller::reset_robot_state()
{
    //Joint state
    for (int i = 0; i < number_of_joints_; i++){
        state_.q(i) = 0.0;
        state_.qd(i) = 0.0;
        state_.qdd(i) = 0.0;
        state_.feedforward_torque(i) = 0.0;
        state_.control_torque(i) = 0.0;
    }

    //Cartesian state - Frames
    for (int i = 0; i < number_of_frames_; i++)
    {
        // KDL::SetToZero(state_.frame_pose[i]);
        // KDL::SetToZero(state_.frame_velocity[i]);
        KDL::SetToZero(state_.frame_acceleration[i]);
    }

    //External forces on the arm's segments
    KDL::Wrench externalForce(
        KDL::Vector(0.0, 0.0, 0.0),  //Linear Force
        KDL::Vector(0.0, 0.0, 0.0)); //Torque

    for (int i = 0; i < number_of_segments_; i++){
        state_.external_force[i] = externalForce;
    }

    //Acceleration Constraints on end-effector
    KDL::Twist unit_constraint_force(
        KDL::Vector(0.0, 0.0, 0.0),  // linear
        KDL::Vector(0.0, 0.0, 0.0)); // angular

    for(int i = 0; i < NUMBER_OF_CONSTRAINTS; i++)
    {
        state_.ee_unit_constraint_forces.setColumn(0, unit_constraint_force);
        state_.ee_effector_acceleration_energy(0) = 0.0;
    }
}

void dynamics_controller::integrate_robot_motion(
    const KDL::JntArray &current_q,
    const KDL::JntArray &current_qd,
    const KDL::JntArray &current_qdd,
    KDL::JntArray &integrated_q,
    KDL::JntArray &integrated_qd,
    std::vector<KDL::Frame> &integratedframe_pose,
    std::vector<KDL::FrameVel> &integrated_frame_velocity,
    int number_of_steps)
{

    double step_size = dt_ * number_of_steps; 

    //Euler method
    for (int i = 0; i < number_of_joints_; i++)
    {
        // std::cout << joint_state[i].angularVelocity.value() << " " << abs(joint_command.data[i]) <<std::endl;
        integrated_qd(i) = current_qd(i) + current_qdd(i) * step_size;
        integrated_q(i) = current_q(i) + (integrated_qd(i) - current_qdd(i) * step_size / 2) * step_size;
    }
}