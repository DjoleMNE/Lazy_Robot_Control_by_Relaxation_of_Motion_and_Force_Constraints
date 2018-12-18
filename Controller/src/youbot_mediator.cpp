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
    // Setting the path for the manipulator configuration file
    config_path_(config_path),
    youbot_arm_("youbot-manipulator", config_path)
{   
    //Resize measurement variables
    q_measured_.resize(NUMBER_OF_JOINTS_);
    qd_measured_.resize(NUMBER_OF_JOINTS_);
    tau_measured_.resize(NUMBER_OF_JOINTS_); 
    // current_measured.resize(NUMBER_OF_JOINTS_);

    //Resize setpoint variables
    q_setpoint_.resize(NUMBER_OF_JOINTS_);
    qd_setpoint_.resize(NUMBER_OF_JOINTS_);
    tau_setpoint_.resize(NUMBER_OF_JOINTS_);    
}

//Get Joint Positions
void youbot_mediator::get_joint_positions(KDL::JntArray &joint_positions) 
{
	youbot_arm_.getJointData(q_measured_);

    // Converting youBot driver joint angles to KDL angles
    for (int i = 0; i < NUMBER_OF_JOINTS_; i++) {
        joint_positions(i) = q_measured_[i].angle.value();
    }
}

//Set Joint Positions
void youbot_mediator::set_joint_positions(const KDL::JntArray &joint_positions)
{
    // Converting KDL angles to youBot driver joint angles 
    for (int i = 0; i < NUMBER_OF_JOINTS_; i++) {
        q_setpoint_[i].angle = joint_positions(i) * radian;
    }    
	youbot_arm_.setJointData(q_setpoint_);
}


//Get Joint Velocities
void youbot_mediator::get_joint_velocities(KDL::JntArray &joint_velocities)
{
	youbot_arm_.getJointData(qd_measured_);
    
    // Converting youBot driver joint velocities to KDL joint velocities
    for (int i = 0; i < NUMBER_OF_JOINTS_; i++) {
        joint_velocities(i) = qd_measured_[i].angularVelocity.value();
    }
}

//Set Joint Velocities
void youbot_mediator::set_joint_velocities(const KDL::JntArray &joint_velocities)
{
    // Converting KDL join velocities to youBot driver joint velocities 
    for (int i = 0; i < NUMBER_OF_JOINTS_; i++) {
        qd_setpoint_[i].angularVelocity = \
                                    joint_velocities(i) * radian_per_second;
    }    
	youbot_arm_.setJointData(qd_setpoint_);
}


//Get Joint Torques
void youbot_mediator::get_joint_torques(KDL::JntArray &joint_torques)
{
	youbot_arm_.getJointData(tau_measured_);
    
    // Converting the youBot driver joint torques to KDL joint torques
    for (int i = 0; i < NUMBER_OF_JOINTS_; i++) {
        joint_torques(i) = tau_measured_[i].torque.value();
    }
}

//Set Joint Torques
void youbot_mediator::set_joint_torques(const KDL::JntArray &joint_torques) 
{

    // Converting KDL joint torques to youBot driver joint torques 
    for (int i = 0; i < NUMBER_OF_JOINTS_; i++) {
        tau_setpoint_[i].torque = joint_torques(i) * newton_meter;
    }    
	youbot_arm_.setJointData(tau_setpoint_);
}

//Extract youBot model from URDF file
int youbot_mediator::get_robot_model(KDL::Chain &arm_chain, 
                                    std::string root_name, 
                                    std::string tooltip_name, 
                                    std::string urdf_path)
{
    //Extract KDL tree from the URDF file
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
int youbot_mediator::initialize(KDL::Chain &arm_chain, 
                                std::string root_name, 
                                std::string tooltip_name,
                                std::string urdf_path)
{
    //Extract youBot model from URDF file
    if(get_robot_model(arm_chain, root_name, tooltip_name, urdf_path) == 0)
        std::cout << "youBot model created successfully! " << std::endl;
    else return -1;

    // Commutate with the joints
    youbot_arm_.doJointCommutation();
    // Calibrate youBot arm
    youbot_arm_.calibrateManipulator();
	std::cout << "youBot initialized successfully! " << std::endl;

    return 0;
}