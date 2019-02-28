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

#include "lwr_rtt_control/lwr_rtt_control.hpp"

LwrRttControl::LwrRttControl(const std::string& name):
RTT::TaskContext(name)
{
    // Here you can add your ports, properties and operations
    // ex : this->addOperation("my_super_function",&LwrRttControl::MyFunction,this,RTT::OwnThread);
    this->addPort("JointPosition",port_joint_position_in).doc("Current joint positions");
    this->addPort("JointVelocity",port_joint_velocity_in).doc("Current joint velocities");
    this->addPort("JointTorque",port_joint_torque_in).doc("Current joint torques");

    this->addPort("JointPositionCommand",port_joint_position_cmd_out).doc("Command joint positions");
    this->addPort("JointVelocityCommand",port_joint_velocity_cmd_out).doc("Command joint velocities");
    this->addPort("JointTorqueCommand",port_joint_torque_cmd_out).doc("Command joint torques");
}

bool LwrRttControl::configureHook()
{
    // Initialize the arm object
    if(!this->arm.init())
    {
        RTT::log(RTT::Fatal)
        << "Could not initialize arm, make sure roscore is launched"
        " and that tip_link, root_link and robot_description are set in rosparam"
        << RTT::endlog();
    }

    // for(int i = 0; i < 7; i++)
    // {
    //     std::cout<< arm.Chain().getSegment(i).getName() << std::endl;
    // }
    
    this->robot_state_= std::make_shared<state_specification>(arm.getNrOfJoints(), arm.getNrOfSegments(), arm.getNrOfSegments() + 1, 6);
    this->hd_solver_ = std::make_shared<KDL::Solver_Vereshchagin>(this->arm.Chain(), 
                                                                  std::vector<double> {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                                                  KDL::Twist(KDL::Vector(0.0, 0.0, 9.81289), KDL::Vector(0.0, 0.0, 0.0)), 
                                                                  6);

    jnt_pos_in.setZero(arm.getNrOfJoints());
    jnt_vel_in.setZero(arm.getNrOfJoints());
    jnt_trq_in.setZero(arm.getNrOfJoints());

    jnt_pos_cmd_out.setZero(arm.getNrOfJoints());
    jnt_vel_cmd_out.setZero(arm.getNrOfJoints());
    jnt_trq_cmd_out.setZero(arm.getNrOfJoints());

    port_joint_position_cmd_out.setDataSample(jnt_pos_cmd_out);
    port_joint_velocity_cmd_out.setDataSample(jnt_vel_cmd_out);
    port_joint_torque_cmd_out.setDataSample(jnt_trq_cmd_out);

    // Check validity of (all) Ports:
    if ( !port_joint_position_in.connected() || 
         !port_joint_velocity_in.connected() ||
         !port_joint_torque_in.connected() ) 
    {
        RTT::log(RTT::Fatal) << "No input connection!"<< RTT::endlog();
        return false;
    }
    if ( !port_joint_position_cmd_out.connected() ||
         !port_joint_torque_cmd_out.connected()) 
    {
           RTT::log(RTT::Warning) << "No output connection!"<< RTT::endlog();  
    }

    robot_state_->q.data << 1.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.0;
    KDL::Twist unit_constraint_force_x(
            KDL::Vector(0.0, 0.0, 0.0),     // linear
            KDL::Vector(0.0, 0.0, 0.0));    // angular
    robot_state_->ee_unit_constraint_force.setColumn(0, unit_constraint_force_x);
    robot_state_->ee_acceleration_energy(0) = 0.0;

    KDL::Twist unit_constraint_force_y(
            KDL::Vector(0.0, 0.0, 0.0),     // linear
            KDL::Vector(0.0, 0.0, 0.0));    // angular
    robot_state_->ee_unit_constraint_force.setColumn(1, unit_constraint_force_y);
    robot_state_->ee_acceleration_energy(1) = 0.0;

    KDL::Twist unit_constraint_force_z(
            KDL::Vector(0.0, 0.0, 0.0),     // linear
            KDL::Vector(0.0, 0.0, 0.0));    // angular
    robot_state_->ee_unit_constraint_force.setColumn(2, unit_constraint_force_z);
    robot_state_->ee_acceleration_energy(2) = 0.0;

    KDL::Twist unit_constraint_force_x1(
            KDL::Vector(0.0, 0.0, 0.0),     // linear
            KDL::Vector(0.0, 0.0, 0.0));    // angular
    robot_state_->ee_unit_constraint_force.setColumn(3, unit_constraint_force_x1);
    robot_state_->ee_acceleration_energy(3) = 0.0;

    KDL::Twist unit_constraint_force_y1(
            KDL::Vector(0.0, 0.0, 0.0),     // linear
            KDL::Vector(0.0, 0.0, 0.0));    // angular
    robot_state_->ee_unit_constraint_force.setColumn(4, unit_constraint_force_y1);
    robot_state_->ee_acceleration_energy(4) = 0.0;

    KDL::Twist unit_constraint_force_z1(
            KDL::Vector(0.0, 0.0, 0.0),     // linear
            KDL::Vector(0.0, 0.0, 0.0));    // angular
    robot_state_->ee_unit_constraint_force.setColumn(5, unit_constraint_force_z1);
    robot_state_->ee_acceleration_energy(5) = 0.0;
    return true;
}

void LwrRttControl::updateHook()
{
    if(!port_joint_position_cmd_out.connected())
    {
        RTT::log(RTT::Warning) << "No position output connection!"<< RTT::endlog();  
    }

    if (!port_joint_torque_cmd_out.connected()) 
    {
        RTT::log(RTT::Warning) << "No torque output connection!"<< RTT::endlog();  
    }

    // Read status from robot
    port_joint_position_in.read(jnt_pos_in);
    port_joint_velocity_in.read(jnt_vel_in);
    port_joint_torque_in.read(jnt_trq_in);

    robot_state_->q.data = jnt_pos_in;
    robot_state_->qd.data = jnt_vel_in;

    // int hd_solver_result = hd_solver_->CartToJnt(robot_state_->q,
    //                                              robot_state_->qd,
    //                                              robot_state_->qdd,
    //                                              robot_state_->ee_unit_constraint_force,
    //                                              robot_state_->ee_acceleration_energy,
    //                                              robot_state_->external_force,
    //                                              robot_state_->feedforward_torque);

    // if(hd_solver_result != 0) printf("ERROR");
    // exit(0);
    // hd_solver_->get_control_torque(robot_state_->control_torque);
    // hd_solver_->get_constraint_torque(robot_state_->control_torque);
    // hd_solver_->get_transformed_link_acceleration(robot_state_->frame_acceleration);

    // std::cout << "Frame ACC" << '\n';
    // for (size_t i = 0; i < 8; i++)
    //     RTT::log(RTT::Warning) << robot_state_->frame_acceleration[i] << RTT::endlog();


    KDL::Vector linearAcc_RNE(0.0, 0.0, -9.81289);
    // KDL::ChainIdSolver_RNE RNE_idsolver(arm.Chain(), linearAcc_RNE);
    // KDL::JntArray control_torque_RNE(arm.getNrOfJoints());
    // KDL::JntArray input_qdd(arm.getNrOfJoints());

    // control_torque_RNE.data = Eigen::VectorXd::Zero(arm.getNrOfJoints());
    // input_qdd.data = Eigen::VectorXd::Zero(arm.getNrOfJoints());

    // int result_RNE = RNE_idsolver.CartToJnt(robot_state_->q,
    //                                         robot_state_->qd,
    //                                         input_qdd,
    //                                         robot_state_->external_force,
    //                                         control_torque_RNE);

    // std::cout <<"\n" <<"Recursive Newton Euler solver" << '\n';
    // RTT::log(RTT::Warning) << "RNE:  "<< control_torque_RNE.data.transpose() << RTT::endlog();
    // assert(result_RNE == 0);

    // KDL::ChainDynParam gravity_solver(arm.Chain(), linearAcc_RNE);
    // KDL::JntArray gravity_torque(arm.getNrOfJoints());
    // gravity_solver.JntToGravity(robot_state_->q, gravity_torque);
    
    // KDL::JntArray coriol_torque(arm.getNrOfJoints());
    // gravity_solver.JntToCoriolis(robot_state_->q,robot_state_->qd, coriol_torque);
 
    // RTT::log(RTT::Warning) << "G     "<< gravity_torque.data.transpose() << RTT::endlog();
    // RTT::log(RTT::Warning) << "C     "<< coriol_torque.data.transpose() << RTT::endlog();
    // RTT::log(RTT::Warning) << "IN    "<< jnt_trq_in.transpose() << RTT::endlog();
 
    KDL::Wrench wrench(KDL::Vector(0.0, 0.0, 0.50),
                        KDL::Vector(0.0, 0.0, 0.0));

    KDL::ChainJntToJacSolver chainjacsolver_(arm.Chain());
    KDL::Jacobian jacob(arm.getNrOfJoints());
    int jac_result = chainjacsolver_.JntToJac(robot_state_->q, jacob);

    Eigen::Matrix<double, 6, 1> w;
    // w << (Eigen::Map<Eigen::Vector3d>(wrench_.force.data), 
    //       Eigen::Map<Eigen::Vector3d>(wrench_.torque.data));

    w(0) = wrench.force(0);
    w(1) = wrench.force(1);
    w(2) = wrench.force(2);
    w(3) = wrench.torque(0);
    w(4) = wrench.torque(1);
    w(5) = wrench.torque(2);

    robot_state_->control_torque.data = jacob.data.transpose() * w;

  
    // jnt_trq_cmd_out = -control_torque_RNE.data;
    // jnt_trq_cmd_out = -gravity_torque.data;
    // jnt_trq_cmd_out += robot_state_->control_torque.data;

    jnt_trq_cmd_out = robot_state_->control_torque.data;
    port_joint_torque_cmd_out.write(jnt_trq_cmd_out);
}

// Let orocos know how to create the component
ORO_CREATE_COMPONENT(LwrRttControl)
