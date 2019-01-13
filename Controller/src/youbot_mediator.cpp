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

youbot_mediator::youbot_mediator(): 
    is_initialized(false), add_offsets_(false), parser_result_(0), 
    connection_established_(false), youbot_model_(youbot_model::URDF),
    youbot_environment_(youbot_environment::SIMULATION),
    linear_root_acc_(youbot_constants::root_acceleration[0],
                        youbot_constants::root_acceleration[1],
                        youbot_constants::root_acceleration[2]),
    angular_root_acc_(youbot_constants::root_acceleration[3],
                        youbot_constants::root_acceleration[4],
                        youbot_constants::root_acceleration[5]),
    root_acc_(linear_root_acc_, angular_root_acc_), 
    robot_chain_(), yb_tree_(), yb_urdf_model_()
{   
    //Resize measurement variables
    q_measured_.resize(youbot_constants::NUMBER_OF_JOINTS);
    qd_measured_.resize(youbot_constants::NUMBER_OF_JOINTS);
    tau_measured_.resize(youbot_constants::NUMBER_OF_JOINTS); 

    //Resize setpoint variables
    q_setpoint_.resize(youbot_constants::NUMBER_OF_JOINTS);
    qd_setpoint_.resize(youbot_constants::NUMBER_OF_JOINTS);
    tau_setpoint_.resize(youbot_constants::NUMBER_OF_JOINTS); 
}

//Get Joint Positions
void youbot_mediator::get_joint_positions(KDL::JntArray &joint_positions) 
{
    if (youbot_environment_ != youbot_environment::SIMULATION)
    {
	    youbot_arm_->getJointData(q_measured_);
        // Converting youBot driver joint angles to KDL angles
        for (int i = 0; i < youbot_constants::NUMBER_OF_JOINTS; i++) 
        {
            joint_positions(i) = q_measured_[i].angle.value();

            // Custom model's home state is not folded - it is candle
            // Check with Sven if the last joint value should be inverted here
            // similary like in the case of getting joint velocities
            // Check JP's code and report
            if (add_offsets_)
            {
                joint_positions(i) = \
                            joint_positions(i) + youbot_constants::joint_offsets[i];
            }
        }
    }

    else // Assign to the current state the previously passed command 
    {
        for (int i = 0; i < youbot_constants::NUMBER_OF_JOINTS; i++)
            joint_positions(i) = q_setpoint_[i].angle.value();
    }
}

//Set Joint Positions
void youbot_mediator::set_joint_positions(const KDL::JntArray &joint_positions)
{
    // Converting KDL angles to youBot driver joint angles 
    for (int i = 0; i < youbot_constants::NUMBER_OF_JOINTS; i++)
    {
        if (add_offsets_)
        {
            q_setpoint_[i].angle = \
                (joint_positions(i) - youbot_constants::joint_offsets[i]) * radian;            
        } 
        
        else q_setpoint_[i].angle = joint_positions(i) * radian;
    }

    if (youbot_environment_ != youbot_environment::SIMULATION)
	    youbot_arm_->setJointData(q_setpoint_);
}

//Get Joint Velocities
void youbot_mediator::get_joint_velocities(KDL::JntArray &joint_velocities)
{
    if (youbot_environment_ != youbot_environment::SIMULATION)
    {
        youbot_arm_->getJointData(qd_measured_);
        
        // Converting youBot driver joint velocities to KDL joint velocities
        for (int i = 0; i < youbot_constants::NUMBER_OF_JOINTS; i++)
            joint_velocities(i) = qd_measured_[i].angularVelocity.value();

        if (add_offsets_) joint_velocities(4) = -1 * joint_velocities(4);
    }

    else // Assign to the current state the previously passed command  
    {
        for (int i = 0; i < youbot_constants::NUMBER_OF_JOINTS; i++)
            joint_velocities(i) = qd_setpoint_[i].angularVelocity.value();
    }
}

//Set Joint Velocities
void youbot_mediator::set_joint_velocities(const KDL::JntArray &joint_velocities)
{   
    // Converting KDL join velocities to youBot driver joint velocities 
    for (int i = 0; i < youbot_constants::NUMBER_OF_JOINTS; i++)
    {
        qd_setpoint_[i].angularVelocity = \
                                    joint_velocities(i) * radian_per_second;
    }

    // if(add_offsets_){
    //     qd_setpoint_[4].angularVelocity = 0.0 * radian_per_second;
    // } 
    if(add_offsets_) 
    {
        qd_setpoint_[4].angularVelocity = \
                                -1 * joint_velocities(4) * radian_per_second;
    }
    
    if (youbot_environment_ != youbot_environment::SIMULATION)
	    youbot_arm_->setJointData(qd_setpoint_);
}

//Get Joint Torques
void youbot_mediator::get_joint_torques(KDL::JntArray &joint_torques)
{
    if (youbot_environment_ != youbot_environment::SIMULATION)
    {
        youbot_arm_->getJointData(tau_measured_);
        
        // Converting the youBot driver joint torques to KDL joint torques
        for (int i = 0; i < youbot_constants::NUMBER_OF_JOINTS; i++)
            joint_torques(i) = tau_measured_[i].torque.value();
        
        if (add_offsets_) joint_torques(4) = -1 * joint_torques(4);
    }

    else // Assign to the current state the previously passed command 
    {
        for (int i = 0; i < youbot_constants::NUMBER_OF_JOINTS; i++)
            joint_torques(i) = tau_setpoint_[i].torque.value();
    }
}

//Set Joint Torques
void youbot_mediator::set_joint_torques(const KDL::JntArray &joint_torques) 
{

    // Converting KDL joint torques to youBot driver joint torques 
    for (int i = 0; i < youbot_constants::NUMBER_OF_JOINTS; i++)
        tau_setpoint_[i].torque = joint_torques(i) * newton_meter;
    
    // if(add_offsets_) tau_setpoint_[4].torque = 0.0 * newton_meter;
    if(add_offsets_) 
        tau_setpoint_[4].torque = -1 * joint_torques(4) * newton_meter;
    
    if (youbot_environment_ != youbot_environment::SIMULATION)
	    youbot_arm_->setJointData(tau_setpoint_);
}

std::vector<double> youbot_mediator::get_maximum_joint_pos_limits()
{
    if(youbot_model_ == youbot_model::YB_STORE) 
        return youbot_constants::joint_position_limits_max_1;

    else 
    {
        if(youbot_environment_ != youbot_environment::SIMULATION)
            return youbot_constants::joint_position_limits_max_2;
        else return youbot_constants::joint_position_limits_max_2_sim;
    }
}

std::vector<double> youbot_mediator::get_minimum_joint_pos_limits()
{
    if(youbot_model_ == youbot_model::YB_STORE) 
        return youbot_constants::joint_position_limits_min_1;
        
    else 
    {
        if(youbot_environment_ != youbot_environment::SIMULATION)
            return youbot_constants::joint_position_limits_min_2;
        else return youbot_constants::joint_position_limits_min_2_sim;
    }
}

std::vector<double> youbot_mediator::get_joint_position_thresholds()
{
    return youbot_constants::joint_position_thresholds;
}

std::vector<double> youbot_mediator::get_joint_velocity_limits()
{
    return youbot_constants::joint_velocity_limits;
}

std::vector<double> youbot_mediator::get_joint_torque_limits()
{
    return youbot_constants::joint_torque_limits;
}

std::vector<double> youbot_mediator::get_joint_inertia()
{
    return youbot_constants::joint_inertia;
}

std::vector<double> youbot_mediator::get_joint_offsets()
{
    return youbot_constants::joint_offsets;
}

KDL::Twist youbot_mediator::get_root_acceleration()
{
    return root_acc_;
}

KDL::Chain youbot_mediator::get_robot_model() 
{
    return robot_chain_; 
}

//Extract youBot model from URDF file
int youbot_mediator::get_model_from_urdf()
{
    //Extract KDL tree from the URDF file
    if (!yb_urdf_model_.initFile(youbot_constants::urdf_path))
    {
        std::cout << "ERROR: Failed to parse urdf robot model" << '\n';
        return -1;
    }

    if (!kdl_parser::treeFromUrdfModel(yb_urdf_model_, yb_tree_))
    {
        std::cout << "ERROR: Failed to construct kdl tree" << '\n';
        return -1;
    }

    //Extract KDL chain from KDL tree
    yb_tree_.getChain(youbot_constants::root_name, 
                      youbot_constants::tooltip_name, 
                      robot_chain_);
    return 0;
}

// Initialize varibles and calibrate the manipulator: 
void youbot_mediator::initialize(const int robot_model,
                                 const int robot_environment)
{
    youbot_model_ = robot_model;
    youbot_environment_ = robot_environment;
    robot_chain_ = KDL::Chain();
    
    // Reset Flags
    is_initialized = false;
    add_offsets_ = false;
    parser_result_ = 0;

    if(youbot_model_ == youbot_model::YB_STORE)
    {
        //Extract model from custom file 
        youbot_custom_model yb_store_model(robot_chain_);
        
        // Add offsets to match real robot and the youBot store model
        if(youbot_environment_ != youbot_environment::SIMULATION) 
            add_offsets_ = true;
    }
    
    //Extract youBot model from the URDF file
    else parser_result_ = get_model_from_urdf();
    
    if (youbot_environment_ != youbot_environment::SIMULATION \
        && !connection_established_)
    {
        this->youbot_arm_ = \
            std::make_shared<youbot::YouBotManipulator>("youbot-manipulator", 
                                                youbot_constants::config_path);
        // Commutate with the joints
        youbot_arm_->doJointCommutation();
        // Calibrate youBot arm
        youbot_arm_->calibrateManipulator();
        connection_established_ = true;
    }
    
    if(parser_result_ != 0){
        std::cout << "Cannot create the youBot model!" << std::endl;
    } else{
        is_initialized = true;
	    std::cout << "youBot initialized successfully! " << std::endl;
    } 
}