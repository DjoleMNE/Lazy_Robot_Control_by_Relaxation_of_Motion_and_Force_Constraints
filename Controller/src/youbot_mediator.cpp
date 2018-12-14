/*
Author(s): Djordje Vukcevic, Sven Schneider
Description: Mediator component for enabling conversion of data types.
Acknowledgment: This sofware component is based on Jeyaprakash Rajagopal's 
master thesis code.

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
#include "youbot_mediator.hpp"

youbot_mediator::youbot_mediator(std::string config_path):
    config_path_(config_path)
{   
    //Resize measurement variables
    q_measured.resize(NUMBER_OF_JOINTS_);
    qd_measured.resize(NUMBER_OF_JOINTS_);
    tau_measured.resize(NUMBER_OF_JOINTS_); 
    // current_measured.resize(NUMBER_OF_JOINTS_);

    //Resize setpoint variables
    q_setpoint.resize(NUMBER_OF_JOINTS_);
    qd_setpoint.resize(NUMBER_OF_JOINTS_);
    tau_setpoint.resize(NUMBER_OF_JOINTS_);    
}

//Get Joint Positions
void youbot_mediator::get_joint_positions(KDL::JntArray &joint_positions) 
{
	youbot_arm->getJointData(q_measured);

    // Converting the youBot driver joint angles to KDL angles
    for (int i = 0; i < NUMBER_OF_JOINTS_; i++) {
        joint_positions(i) = q_measured[i].angle.value();
    }
}

//Set Joint Positions
void youbot_mediator::set_joint_positions(const KDL::JntArray &joint_positions)
{

    // Converting KDL angles to youBot driver joint angles 
    for (int i = 0; i < NUMBER_OF_JOINTS_; i++) {
        q_setpoint[i].angle = joint_positions(i) * radian;
    }    
	youbot_arm->setJointData(q_setpoint);
}


//Get Joint Velocities
void youbot_mediator::get_joint_velocities(KDL::JntArray &joint_velocities)
{
	youbot_arm->getJointData(qd_measured);
    
    // Converting the youBot driver joint velocities to KDL velocities
    for (int i = 0; i < NUMBER_OF_JOINTS_; i++) {
        joint_velocities(i) = qd_measured[i].angularVelocity.value();
    }
}

//Set Joint Velocities
void youbot_mediator::set_joint_velocities(const KDL::JntArray &joint_velocities)
{

    // Converting KDL velocities to youBot driver joint velocities 
    for (int i = 0; i < NUMBER_OF_JOINTS_; i++) {
        qd_setpoint[i].angularVelocity = \
                                    joint_velocities(i) * radian_per_second;
    }    
	youbot_arm->setJointData(qd_setpoint);
}


//Get Joint Torques
void youbot_mediator::get_joint_torques(KDL::JntArray &joint_torques)
{
	youbot_arm->getJointData(tau_measured);
    
    // Converting the youBot driver joint torques to KDL torques
    for (int i = 0; i < NUMBER_OF_JOINTS_; i++) {
        joint_torques(i) = tau_measured[i].torque.value();
    }
}

//Set Joint Torques
void youbot_mediator::set_joint_torques(const KDL::JntArray &joint_torques) 
{

    // Converting KDL torques to youBot driver joint torques 
    for (int i = 0; i < NUMBER_OF_JOINTS_; i++) {
        tau_setpoint[i].torque = joint_torques(i) * newton_meter;
    }    
	youbot_arm->setJointData(tau_setpoint);
}

//Extract youBot model from URDF file
int youbot_mediator::get_robot_model(KDL::Chain &arm_chain, 
                                    std::string root_name, 
                                    std::string tooltip_name, 
                                    std::string urdf_path)
{
    //Extract KDL tree from URDF file
    KDL::Tree yb_tree;
    urdf::Model yb_model;

    if (!yb_model.initFile(urdf_path))
    {
        std::cout << "ERROR: Failed to parse urdf robot model" << '\n';
        return -1;
    }

    if (!kdl_parser::treeFromUrdfModel(yb_model, yb_tree))
    {
        std::cout << "ERROR: Failed to construct kdl tree" << '\n';
        return -1;
    }

    //Extract KDL chain from KDL tree
    yb_tree.getChain(root_name, tooltip_name, arm_chain);
    return 0;
}

// Initialize varibles and calibrate the manipulator: 
void youbot_mediator::initialize(KDL::Chain &arm_chain, 
                                std::string root_name, 
                                std::string tooltip_name,
                                std::string urdf_path)
{
    //Extract youBot model from URDF file
    if(get_robot_model(arm_chain, root_name, tooltip_name, urdf_path) == 0)
        std::cout << "youBot model created successfully! " << std::endl;

    // Setting the path for the manipulator configuration file
    this->youbot_arm = std::make_shared<youbot::YouBotManipulator>("youbot-manipulator", config_path_);

    // Commutate with the joints
    youbot_arm->doJointCommutation();
    // Calibrate youBot arm
    youbot_arm->calibrateManipulator();
	std::cout << "youBot initialized successfully! " << std::endl;
}