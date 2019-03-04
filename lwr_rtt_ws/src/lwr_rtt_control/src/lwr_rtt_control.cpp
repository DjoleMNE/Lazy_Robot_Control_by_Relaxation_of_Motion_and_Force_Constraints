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
const int MILLISECOND = 1000;
const long SECOND = 1000000;

LwrRttControl::LwrRttControl(const std::string& name):
    RTT::TaskContext(name), environment_(lwr_environment::LWR_SIMULATION), 
    robot_model_(lwr_model::LWR_URDF), RATE_HZ_(999), NUM_OF_SEGMENTS_(7), 
    NUM_OF_JOINTS_(7), NUM_OF_CONSTRAINTS_(6), 
    robot_state_(NUM_OF_JOINTS_, NUM_OF_SEGMENTS_, NUM_OF_SEGMENTS_ + 1, NUM_OF_CONSTRAINTS_)
{
    // Here you can add your ports, properties and operations
    // ex : this->addOperation("my_super_function",&LwrRttControl::MyFunction,this,RTT::OwnThread);
    this->addPort("JointPosition",port_joint_position_in).doc("Current joint positions");
    this->addPort("JointVelocity",port_joint_velocity_in).doc("Current joint velocities");
    this->addPort("JointTorque",port_joint_torque_in).doc("Current joint torques");

    this->addPort("JointPositionCommand",port_joint_position_cmd_out).doc("Command joint positions");
    this->addPort("JointVelocityCommand",port_joint_velocity_cmd_out).doc("Command joint velocities");
    this->addPort("JointTorqueCommand",port_joint_torque_cmd_out).doc("Command joint torques");
//     this->addProperty("environment", environment_).doc("environment");
//     this->addProperty("robot_model", robot_model_).doc("robot_model");
    this->addProperty("compensate_gravity", compensate_gravity_).doc("compensate gravity");

}

bool LwrRttControl::configureHook()
{
    rtt_ros_kdl_tools::getAllPropertiesFromROSParam(this);

    jnt_pos_in.setZero(NUM_OF_JOINTS_);
    jnt_vel_in.setZero(NUM_OF_JOINTS_);
    jnt_trq_in.setZero(NUM_OF_JOINTS_);

    jnt_pos_cmd_out.setZero(NUM_OF_JOINTS_);
    jnt_vel_cmd_out.setZero(NUM_OF_JOINTS_);
    jnt_trq_cmd_out.setZero(NUM_OF_JOINTS_);
    jnt_gravity_trq_out.data.setZero(NUM_OF_JOINTS_);

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

    robot_driver_.initialize(robot_model_, environment_, compensate_gravity_);
    assert(NUM_OF_JOINTS_ == robot_driver_.get_robot_model().getNrOfSegments());

    this->gravity_solver_ = std::make_shared<KDL::ChainDynParam>(robot_driver_.get_robot_model(), 
                                                                 KDL::Vector(0.0, 0.0, -9.81289)); 

    this->controller_ = std::make_shared<dynamics_controller>(&robot_driver_, RATE_HZ_);

    //Create End_effector Cartesian Acceleration task 
    controller_->define_ee_acc_constraint(std::vector<bool>{false, false, false, // Linear
                                                            false, false, false}, // Angular
                                          std::vector<double>{0.0, 0.0, 0.0, // Linear
                                                              0.0, 0.0, 0.0}); // Angular
    //Create External Forces task 
    controller_->define_ee_external_force(std::vector<double>{0.0, 0.0, 0.0, // Linear
                                                              0.0, 0.0, 0.0}); // Angular
    //Create Feedforward torques task s
    controller_->define_feedforward_torque(std::vector<double>{0.0, 0.0, 
                                                               0.0, 0.0, 
                                                               0.0, 0.0, 0.0}); 

    controller_->define_desired_ee_pose(std::vector<bool>{true, true, true, // Linear
                                                          false, false, false}, // Angular
                                        std::vector<double>{-0.210785, -0.328278,  0.632811, // Linear: Vector
                                                            -0.540302, -0.841471, -0.000860, // Angular: Rotation matrix
                                                            -0.841470,  0.540302, -0.001340,
                                                             0.001592,  0.000000, -0.999999});
//     controller_->control(control_mode::TORQUE, true);
    controller_->initialize(control_mode::TORQUE, true);

    return true;
}

void LwrRttControl::updateHook()
{
    // Read status from robot
    port_joint_position_in.read(jnt_pos_in);
    port_joint_velocity_in.read(jnt_vel_in);
    port_joint_torque_in.read(jnt_trq_in);
 
    robot_state_.q.data  = jnt_pos_in;
    robot_state_.qd.data = jnt_vel_in;

    controller_->step(robot_state_.q.data, 
                      robot_state_.qd.data, 
                      robot_state_.control_torque.data);

    gravity_solver_->JntToGravity(robot_state_.q, jnt_gravity_trq_out);

    if(compensate_gravity_) jnt_trq_cmd_out = robot_state_.control_torque.data;
    else jnt_trq_cmd_out = robot_state_.control_torque.data - jnt_gravity_trq_out.data;

    port_joint_torque_cmd_out.write(jnt_trq_cmd_out);
}

void LwrRttControl::stopHook()
{
    RTT::log(RTT::Warning) << "Robot stopped!" << RTT::endlog();
    controller_->deinitialize();
}

// Let orocos know how to create the component
ORO_CREATE_COMPONENT(LwrRttControl)
