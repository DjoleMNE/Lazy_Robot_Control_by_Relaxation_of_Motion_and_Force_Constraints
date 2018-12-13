// #include <kdl_parser/kdl_parser.hpp>
// #include <youbot_driver/youbot/YouBotManipulator.hpp>
// #include <urdf/model.h>
// #include <dynamics_controller.hpp>
// #include <iostream>
// #include <unistd.h>
// #include <cmath>
// #include <stdlib.h>     /* abs */

// void init_motion(const KDL::Chain &arm_chain_,
//                  motion_specification &motion_,
//                  const int NUMBER_OF_CONSTRAINTS)
// {

//     int number_of_segments = arm_chain_.getNrOfSegments();
//     int number_of_joints = arm_chain_.getNrOfJoints();

//     motion_.set_motion(number_of_joints,
//                         number_of_segments,
//                         NUMBER_OF_CONSTRAINTS);

//     // external forces on the arm (Not including tool segment)
//     for (int i = 0; i < number_of_segments; i++) {
//         KDL::Wrench externalForce(
//            KDL::Vector(0.0, 0.0, 0.0), //Linear Force
//            KDL::Vector(0.0, 0.0, 0.0)); //Torque
//         motion_.external_force[i] = externalForce;
//     }

//     for (int i = 0; i < number_of_joints; i++) motion_.q(i) = 0.0;
//     for (int i = 0; i < number_of_joints; i++) motion_.qd(i) = 0.0;
//     for (int i = 0; i < number_of_joints; i++) motion_.qdd(i) = 0.0;
//     for (int i = 0; i < number_of_joints; i++) motion_.feedforward_torque(i) = 0.0;
// }

// void define_ee_task(motion_specification &motion_)
// {
//     //Stand still command
//     KDL::Twist unit_constraint_force_xl(
//         KDL::Vector(0.0, 0.0, 0.0),  // linear
//         KDL::Vector(0.0, 0.0, 0.0)); // angular
//     motion_.end_effector_unit_constraint_forces.setColumn(0, unit_constraint_force_xl);
//     motion_.end_effector_acceleration_energy_setpoint(0) = 0.0001;

//     KDL::Twist unit_constraint_force_yl(
//         KDL::Vector(0.0, 0.0, 0.0),  // linear
//         KDL::Vector(0.0, 0.0, 0.0)); // angular
//     motion_.end_effector_unit_constraint_forces.setColumn(1, unit_constraint_force_yl);
//     motion_.end_effector_acceleration_energy_setpoint(1) = 0.0;

//     KDL::Twist unit_constraint_force_zl(
//         KDL::Vector(0.0, 0.0, 1.0),  // linear
//         KDL::Vector(0.0, 0.0, 0.0)); // angular
//     motion_.end_effector_unit_constraint_forces.setColumn(2, unit_constraint_force_zl);
//     motion_.end_effector_acceleration_energy_setpoint(2) = 0.0001;
//     //
//     KDL::Twist unit_constraint_force_xa(
//         KDL::Vector(0.0, 0.0, 0.0),  // linear
//         KDL::Vector(0.0, 0.0, 0.0)); // angular
//     motion_.end_effector_unit_constraint_forces.setColumn(3, unit_constraint_force_xa);
//     motion_.end_effector_acceleration_energy_setpoint(3) = 0.0;

//     KDL::Twist unit_constraint_force_ya(
//         KDL::Vector(0.0, 0.0, 0.0),  // linear
//         KDL::Vector(0.0, 0.0, 0.0)); // angular
//     motion_.end_effector_unit_constraint_forces.setColumn(4, unit_constraint_force_ya);
//     motion_.end_effector_acceleration_energy_setpoint(4) = 0.0;

//     KDL::Twist unit_constraint_force_za(
//         KDL::Vector(0.0, 0.0, 0.0),  // linear
//         KDL::Vector(0.0, 0.0, 0.0)); // angular
//     motion_.end_effector_unit_constraint_forces.setColumn(5, unit_constraint_force_za);
//     motion_.end_effector_acceleration_energy_setpoint(5) = 0.0;

//     // KDL::Wrench externalForceEE(KDL::Vector(0.0,
//     //                                         0.0,
//     //                                         0.0), //Linear Force
//     //                             KDL::Vector(0.0,
//     //                                         0.0,
//     //                                         0.0)); //Torque

//     // motion_.external_force[motion_.external_force.size() - 1] = externalForceEE;
// }

// int extract_robot_model(KDL::Chain &arm_chain_, std::string root_name, std::string tooltip_name)
// {
//     //Extract KDL tree from URDF file
//     KDL::Tree yb_tree;
//     urdf::Model yb_model;

//     if (!yb_model.initFile("/home/djole/Master/Thesis/GIT/MT_testing/Main_Code/urdf/youbot_arm_only.urdf"))
//     {
//         std::cout << "ERROR: Failed to parse urdf robot model" << '\n';
//         return -1;
//     }

//     if (!kdl_parser::treeFromUrdfModel(yb_model, yb_tree))
//     {
//         std::cout << "ERROR: Failed to construct kdl tree" << '\n';
//         return -1;
//     }

//     //Extract KDL chain from KDL tree
//     yb_tree.getChain(root_name, tooltip_name, arm_chain_);

//     return 0;
// }

// void integrate_joints(KDL::JntArray &joint_command,
//                       std::vector<youbot::JointSensedVelocity> &joint_state,
//                       std::vector<youbot::JointVelocitySetpoint> &integrated_data,
//                       double joint_limit[],
//                       double dt)
// {
//     double integrated_value;
//     for (int i = 0; i < integrated_data.size(); i++)


//     {
//         // std::cout << joint_state[i].angularVelocity.value() << " " << abs(joint_command.data[i]) <<std::endl;

//         integrated_value = joint_state[i].angularVelocity.value() + joint_command.data[i] * dt;

//         //If joint limit reached, stop the program
//         if (abs(integrated_value) > joint_limit[i]){
//             std::cout << abs(integrated_value) <<" "<< i << std::endl;
//         }
        
//         assert(abs(integrated_value) <= joint_limit[i]);
//         integrated_data[i].angularVelocity = integrated_value * radian_per_second;

//     }
// }


// int main(int argc, char **argv)
// {
//     const int JOINTS = 5;
//     const int NUMBER_OF_CONSTRAINTS = 6;
//     const int MILLISECOND = 1000;


//     KDL::Chain arm_chain_;
//     motion_specification motion_;
//     assert (extract_robot_model(arm_chain_, "arm_link_0", "arm_link_5") != -1);
//     double joint_limit[JOINTS] = {1.5707, 0.8, 1.0, 1.5707, 1.5707};

//     std::vector<youbot::JointSensedAngle> q(JOINTS);
//     std::vector<youbot::JointSensedVelocity> qd(JOINTS);
//     std::vector<youbot::JointSensedTorque> tau(JOINTS);
//     std::vector<youbot::JointSensedCurrent> current(JOINTS);
//     std::vector<youbot::JointVelocitySetpoint> qd_setpoint(JOINTS);
//     std::vector<youbot::JointTorqueSetpoint> tau_setpoint(JOINTS);

//     //Set initial values for setpoints to 0 value
//     for (int i = 0; i < JOINTS; i++)
//     {
//         qd_setpoint[i].angularVelocity = 0 * radian_per_second;
//         tau_setpoint[i].torque = 0 * newton_meter;
//     }

//     int number_of_segments = arm_chain_.getNrOfSegments();
//     int number_of_joints = arm_chain_.getNrOfJoints();

//     assert(JOINTS == number_of_segments);

//     init_motion(arm_chain_, motion_, NUMBER_OF_CONSTRAINTS);
//     define_ee_task(motion_);

//     //arm root acceleration
//     KDL::Vector linearAcc(0.0, 0.0, -9.81); //gravitational acceleration along Z
//     KDL::Vector angularAcc(0.0, 0.0, 0.0);
//     KDL::Twist root_acc(linearAcc, angularAcc);

//     KDL::Solver_Vereshchagin hd_solver_(arm_chain_, root_acc,
//                                         NUMBER_OF_CONSTRAINTS);

//     youbot::YouBotManipulator arm("youbot-manipulator", "/home/djole/Master/Thesis/GIT/MT_testing/youbot_driver/config");
//     arm.doJointCommutation();
//     arm.calibrateManipulator();

//     arm.getJointData(q);
//     arm.getJointData(qd);

//     // std::cout << "\n"
//     //           << "Joint Positions"
//     //           << std::endl;

//     // for (int i = 0; i < JOINTS; i++){
//     //     std::cout << q[i].angle.value() << ",";
//     // }

//     // std::cout << "\n" <<"Joint Velocities"<< std::endl;

//     // for (int i = 0; i < JOINTS; i++)
//     // {
//     //     std::cout << qd[i].angularVelocity.value() << ",";
//     // }

//     // double candle[] = {2.1642, 1.13446, -2.54818, 1.78896, 0.12};
//     // std::vector<youbot::JointAngleSetpoint> candle_set_point(JOINTS);
//     // for (int i = 0; i < JOINTS; i++)
//     //     candle_set_point[i].angle = candle[i] * radian;
//     // arm.setJointData(candle_set_point);

//     // double navigation[] = {2.9496, 1.0, -1.53240, 2.85214, 2.93816};
//     // double navigation[] = {2.9496, 0.075952, -1.53240, 3.35214, 2.93816};
//     // std::vector<youbot::JointAngleSetpoint> navigation_set_point(JOINTS);
//     // for (int i = 0; i < JOINTS; i++)
//     //     navigation_set_point[i].angle = navigation[i] * radian;
//     // arm.setJointData(navigation_set_point);

//     double folded[] = {0.02, 0.02, -0.02, 0.023, 0.12};
//     std::vector<youbot::JointAngleSetpoint> folded_set_point(JOINTS);
//     for (int i = 0; i < JOINTS; i++)
//         folded_set_point[i].angle = folded[i] * radian;
//     arm.setJointData(folded_set_point);

//     std::vector<KDL::Twist> frame_acceleration_;
//     frame_acceleration_.resize(arm_chain_.getNrOfSegments() + 1);

//     //loop with Hz
//     double rate = MILLISECOND;

//     //Time sampling interval
//     double dt = 1.0 / rate;
//     std::cout<<"dt: " << dt << "\n";
//     usleep(5000 * MILLISECOND);

//     while (true)
//     {
//         arm.getJointData(q);
//         arm.getJointData(qd);

//         for (int i = 0; i < JOINTS; i++)
//         {
//             motion_.q(i) = q[i].angle.value();
//             motion_.qd(i) = qd[i].angularVelocity.value();
//             motion_.qdd(i) = 0.0;
//         }

//         // std::cout << "Joints  Pos: " << motion_.q << '\n';
//         // std::cout << "Joints  Vel: " << motion_.qd << '\n';

//         int result = hd_solver_.CartToJnt(motion_.q,
//                                           motion_.qd,
//                                           motion_.qdd,                                       //qdd_ is overwritten by resulting acceleration
//                                           motion_.end_effector_unit_constraint_forces,       // alpha
//                                           motion_.end_effector_acceleration_energy_setpoint, // beta
//                                           motion_.external_force,
//                                           motion_.feedforward_torque);

//         // std::cout << "Solver return: " << result << '\n';
//         // std::cout << "Joints  Acc: " << motion_.qdd << '\n' << "\n";
//         assert(result == 0);

//         integrate_joints(motion_.qdd, qd, qd_setpoint, joint_limit, dt);

//         // std::cout << "\n Joint Vel::Commanded: ";
//         // for (int i = 0; i < JOINTS; i++)
//         // {
//         //     std::cout << qd_setpoint[i].angularVelocity.value() << " , ";
//         // }
//         // std::cout << "\n";

//         // hd_solver_.get_transformed_link_acceleration(frame_acceleration_);

//         // std::cout << "\n \n Frame ACC" << '\n';
//         // for (size_t i = 0; i < arm_chain_.getNrOfSegments() + 1; i++)
//         // {
//         //     std::cout << frame_acceleration_[i] << '\n';
//         // }

//         // KDL::JntArray control_torque_Ver(number_of_joints);
//         // hd_solver_.get_control_torque(control_torque_Ver);
//         // std::cout << "\n"
//         //           << "Joint torques:        " << control_torque_Ver << '\n';

//         // std::cout << "\n";

//         arm.setJointData(qd_setpoint);

//         // stopMotion();

//         usleep(MILLISECOND);
//     }

//     //Linux sleep 
//     // usleep(5000 * MILLISECOND);

//     // for (int i = 0; i < JOINTS; i++)
//     // {
//     //     qd_setpoint[i].angularVelocity = 0.0 * radian_per_second;
//     // }

//     // arm.setJointData(qd_setpoint);

//     return 0;
// }
