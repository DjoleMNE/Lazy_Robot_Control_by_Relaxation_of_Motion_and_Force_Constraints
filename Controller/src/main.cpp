#include <youbot_mediator.hpp>
#include <iostream>
#include <sstream>
#include <fstream>
#include <time.h>
#include <cmath>
#include <boost/assign/list_of.hpp>
#include <stdlib.h> /* abs */
#include <unistd.h>

void initialize_state(const KDL::Chain &arm_chain_,
                 state_specification &motion_,
                 const int NUMBER_OF_CONSTRAINTS)
{

    int number_of_segments = arm_chain_.getNrOfSegments();
    int number_of_joints = arm_chain_.getNrOfJoints();

    motion_.init_state(number_of_joints,
                       number_of_segments,
                       number_of_segments + 1,
                       NUMBER_OF_CONSTRAINTS);

    // external forces on the arm
    for (int i = 0; i < number_of_segments; i++)
    {
        KDL::Wrench externalForce(
            KDL::Vector(0.0, 0.0, 0.0),  //Linear Force
            KDL::Vector(0.0, 0.0, 0.0)); //Torque
        motion_.external_force[i] = externalForce;
    }

    for (int i = 0; i < number_of_joints; i++){
        motion_.q(i) = 0.0;
        motion_.qd(i) = 0.0;
        motion_.qdd(i) = 0.0;
        motion_.feedforward_torque(i) = 0.0;
    }

}

void define_ee_task(state_specification &motion_)
{
    KDL::Twist unit_constraint_force_xl(
        KDL::Vector(0.0, 0.0, 0.0),  // linear
        KDL::Vector(0.0, 0.0, 0.0)); // angular
    motion_.ee_unit_constraint_forces.setColumn(0, unit_constraint_force_xl);
    motion_.ee_acceleration_energy(0) = 0.0001;

    KDL::Twist unit_constraint_force_yl(
        KDL::Vector(0.0, 0.0, 0.0),  // linear
        KDL::Vector(0.0, 0.0, 0.0)); // angular
    motion_.ee_unit_constraint_forces.setColumn(1, unit_constraint_force_yl);
    motion_.ee_acceleration_energy(1) = 0.0;

    KDL::Twist unit_constraint_force_zl(
        KDL::Vector(0.0, 0.0, 1.0),  // linear
        KDL::Vector(0.0, 0.0, 0.0)); // angular
    motion_.ee_unit_constraint_forces.setColumn(2, unit_constraint_force_zl);
    motion_.ee_acceleration_energy(2) = 0.000001;
    //
    KDL::Twist unit_constraint_force_xa(
        KDL::Vector(0.0, 0.0, 0.0),  // linear
        KDL::Vector(0.0, 0.0, 0.0)); // angular
    motion_.ee_unit_constraint_forces.setColumn(3, unit_constraint_force_xa);
    motion_.ee_acceleration_energy(3) = 0.0;

    KDL::Twist unit_constraint_force_ya(
        KDL::Vector(0.0, 0.0, 0.0),  // linear
        KDL::Vector(0.0, 0.0, 0.0)); // angular
    motion_.ee_unit_constraint_forces.setColumn(4, unit_constraint_force_ya);
    motion_.ee_acceleration_energy(4) = 0.0;

    KDL::Twist unit_constraint_force_za(
        KDL::Vector(0.0, 0.0, 0.0),  // linear
        KDL::Vector(0.0, 0.0, 0.0)); // angular
    motion_.ee_unit_constraint_forces.setColumn(5, unit_constraint_force_za);
    motion_.ee_acceleration_energy(5) = 0.0;

    // KDL::Wrench externalForceEE(KDL::Vector(0.0,
    //                                         0.0,
    //                                         0.0), //Linear Force
    //                             KDL::Vector(0.0,
    //                                         0.0,
    //                                         0.0)); //Torque

    // motion_.external_force[motion_.external_force.size() - 1] = externalForceEE;
}

void integrate_joints(const state_specification &current_state,
                    command_specification &commanded_state,
                    double joint_limit[],
                    double dt)
{
    double integrated_value;
    for (int i = 0; i < 5; i++)
    {
        // std::cout << joint_state[i].angularVelocity.value() << " " << abs(joint_command.data[i]) <<std::endl;

        integrated_value = current_state.qd(i) + current_state.qdd(i) * dt;

        //If joint limit reached, stop the program
        if (abs(integrated_value) > joint_limit[i])
        {
            std::cout <<"Limit reached on: " << abs(integrated_value) 
                        << " " << i << std::endl;
        }

        assert(abs(integrated_value) <= joint_limit[i]);
        commanded_state.qd_setpoint(i) = integrated_value;
    }
}

int main(int argc, char **argv)
{
    const int JOINTS = 5;
    const int NUMBER_OF_CONSTRAINTS = 6;
    const int MILLISECOND = 1000;
    double joint_limit[JOINTS] = {1.5707, 0.8, 1.0, 1.5707, 1.5707};
    
    KDL::Chain arm_chain_;
    state_specification motion_;
    command_specification commands_;
    youbot_mediator arm("/home/djole/Master/Thesis/GIT/MT_testing/youbot_driver/config");
    
    arm.initialize(arm_chain_, "arm_link_0", "arm_link_5",
        "/home/djole/Master/Thesis/GIT/MT_testing/Controller/urdf/youbot_arm_only.urdf");
    initialize_state(arm_chain_, motion_, NUMBER_OF_CONSTRAINTS);

    int number_of_segments = arm_chain_.getNrOfSegments();
    int number_of_joints = arm_chain_.getNrOfJoints();
    assert(JOINTS == number_of_segments);
    commands_.init_commands(number_of_joints);

    //Set initial values for setpoints to 0 value
    arm.set_joint_velocities(motion_.qd);
    
    //Create End_effector Cartesian Acceleration task 
    define_ee_task(motion_);

    //arm root acceleration
    KDL::Vector linearAcc(0.0, 0.0, -9.81); //gravitational acceleration along Z
    KDL::Vector angularAcc(0.0, 0.0, 0.0);
    KDL::Twist root_acc(linearAcc, angularAcc);

    KDL::Solver_Vereshchagin hd_solver_(arm_chain_, root_acc,
                                        NUMBER_OF_CONSTRAINTS);

    // Go to Candle configuration  
    // KDL::JntArray candle_pose(JOINTS);
    // double candle[] = {2.1642, 1.13446, -2.54818, 1.78896, 0.12};
    // for (int i = 0; i < JOINTS; i++) candle_pose(i) = candle[i];  
    // arm.set_joint_positions(candle_pose);

    // double navigation[] = {2.9496, 1.0, -1.53240, 2.85214, 2.93816};
    // double navigation[] = {2.9496, 0.075952, -1.53240, 3.35214, 2.93816};
    // std::vector<youbot::JointAngleSetpoint> navigation_set_point(JOINTS);
    // for (int i = 0; i < JOINTS; i++)
    //     navigation_set_point[i].angle = navigation[i] * radian;
    // arm.setJointData(navigation_set_point);

    // double folded[] = {0.02, 0.02, -0.02, 0.023, 0.12};
    // std::vector<youbot::JointAngleSetpoint> folded_set_point(JOINTS);
    // for (int i = 0; i < JOINTS; i++)
    //     folded_set_point[i].angle = folded[i] * radian;
    // arm.setJointData(folded_set_point);

    arm.get_joint_positions(motion_.q);
    arm.get_joint_velocities(motion_.qd);

    std::cout << "\n"
              << "Joint Positions" << motion_.q
              << std::endl;

    std::cout << "\n" <<"Joint Velocities"<< motion_.qd << std::endl;

    std::vector<KDL::Twist> frame_acceleration_;
    frame_acceleration_.resize(arm_chain_.getNrOfSegments() + 1);

    //loop with Hz
    double rate = MILLISECOND;

    //Time sampling interval
    double dt = 1.0 / rate;
    // std::cout << "dt: " << dt << "\n";

    usleep(5000 * MILLISECOND);
    // return 0;

    while (1)
    {
        arm.get_joint_positions(motion_.q);
        arm.get_joint_velocities(motion_.qd);

        // std::cout << "Joints  Pos: " << motion_.q << '\n';
        // std::cout << "Joints  Vel: " << motion_.qd << '\n';

        int result = hd_solver_.CartToJnt(motion_.q,
                                          motion_.qd,
                                          motion_.qdd,                                       //qdd_ is overwritten by resulting acceleration
                                          motion_.ee_unit_constraint_forces,       // alpha
                                          motion_.ee_acceleration_energy, // beta
                                          motion_.external_force,
                                          motion_.feedforward_torque);

        // std::cout << "Solver return: " << result << '\n';
        // std::cout << "Joints  Acc: " << motion_.qdd << '\n' << "\n";
        assert(result == 0);

        integrate_joints(motion_, commands_, joint_limit, dt);

        // std::cout << "\n Joint Vel::Commanded: ";
        // for (int i = 0; i < JOINTS; i++)
        // {
        //     std::cout << qd_setpoint[i].angularVelocity.value() << " , ";
        // }
        // std::cout << "\n";

        hd_solver_.get_transformed_link_acceleration(frame_acceleration_);

        // std::cout << "\n \n Frame ACC" << '\n';
        // for (size_t i = 0; i < arm_chain_.getNrOfSegments() + 1; i++)
        // {
        //     std::cout << frame_acceleration_[i] << '\n';
        // }

        // KDL::JntArray control_torque_Ver(number_of_joints);
        // hd_solver_.get_control_torque(control_torque_Ver);
        // std::cout << "\n"
        //           << "Joint torques:        " << control_torque_Ver << '\n';

        // std::cout << "\n";

        arm.set_joint_velocities(commands_.qd_setpoint);
        usleep(MILLISECOND);
    }
    return 0;
}
