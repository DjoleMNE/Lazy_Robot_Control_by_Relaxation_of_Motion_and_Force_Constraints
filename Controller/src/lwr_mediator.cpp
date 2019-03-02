/*
Author(s): Djordje Vukcevic, Sven Schneider
Description: Mediator component for enabling conversion of data types.
Acknowledgment: This sofware component is based on Jeyaprakash Rajagopal's 
master thesis code.

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
#include "lwr_mediator.hpp"

lwr_mediator::lwr_mediator(): 
    is_initialized_(false), 
    // ROBOT_ID_(youbot_constants::ID), add_offsets_(false),
    // parser_result_(0), connection_established_(false), 
    // youbot_model_(youbot_model::URDF),
    // youbot_environment_(youbot_environment::SIMULATION),
    // linear_root_acc_(youbot_constants::root_acceleration[0],
    //                     youbot_constants::root_acceleration[1],
    //                     youbot_constants::root_acceleration[2]),
    // angular_root_acc_(youbot_constants::root_acceleration[3],
    //                     youbot_constants::root_acceleration[4],
    //                     youbot_constants::root_acceleration[5]),
    // root_acc_(linear_root_acc_, angular_root_acc_), 
    lwr_chain_(), lwr_tree_(), lwr_urdf_model_()
{   

}

//Get Joint Positions
void lwr_mediator::get_joint_positions(KDL::JntArray &joint_positions) 
{

}

//Set Joint Positions
void lwr_mediator::set_joint_positions(const KDL::JntArray &joint_positions)
{

}

//Get Joint Velocities
void lwr_mediator::get_joint_velocities(KDL::JntArray &joint_velocities)
{

}

//Set Joint Velocities
void lwr_mediator::set_joint_velocities(const KDL::JntArray &joint_velocities)
{   

}

//Get Joint Torques
void lwr_mediator::get_joint_torques(KDL::JntArray &joint_torques)
{

}

//Set Joint Torques
void lwr_mediator::set_joint_torques(const KDL::JntArray &joint_torques) 
{

}

void lwr_mediator::set_joint_command(const KDL::JntArray &joint_positions,
                                        const KDL::JntArray &joint_velocities,
                                        const KDL::JntArray &joint_torques,
                                        const int desired_control_mode)
{

}

std::vector<double> lwr_mediator::get_maximum_joint_pos_limits()
{

}

std::vector<double> lwr_mediator::get_minimum_joint_pos_limits()
{

}

std::vector<double> lwr_mediator::get_joint_position_thresholds()
{
}

std::vector<double> lwr_mediator::get_joint_velocity_limits()
{
}

std::vector<double> lwr_mediator::get_joint_torque_limits()
{
}

std::vector<double> lwr_mediator::get_joint_inertia()
{
}

std::vector<double> lwr_mediator::get_joint_offsets()
{
}

KDL::Twist lwr_mediator::get_root_acceleration()
{
    return root_acc_;
}

KDL::Chain lwr_mediator::get_robot_model() 
{
    return lwr_chain_; 
}

//Extract youBot model from URDF file
int lwr_mediator::get_model_from_urdf()
{
    return 0;
}

std::string lwr_mediator::get_robot_ID()
{
    return ROBOT_ID_;
}

bool lwr_mediator::is_initialized()
{
    return is_initialized_;
}

// Initialize variables and calibrate the manipulator: 
void lwr_mediator::initialize(const int robot_model, const int robot_environment)
{
    lwr_model_ = robot_model;
    lwr_environment_ = robot_environment;
    lwr_chain_ = KDL::Chain();
    
    // Reset Flags
    is_initialized_ = false;
    add_offsets_ = false;
    parser_result_ = 0;

    // if(lwr_model_ == lwr_model::YB_STORE)
    
    //Extract youBot model from the URDF file
    // else parser_result_ = get_model_from_urdf();
    
    if (lwr_environment_ != lwr_environment::LWR_SIMULATION && !connection_established_)
    {
        connection_established_ = true;
    }
    
    if(parser_result_ != 0){
        std::cout << "Cannot create the youBot model!" << std::endl;
    } else{
        is_initialized_ = true;
	    std::cout << "youBot initialized successfully! " << std::endl;
    } 
}