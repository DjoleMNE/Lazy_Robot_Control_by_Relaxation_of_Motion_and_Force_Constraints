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
    ROBOT_ID_(lwr_constants::ID), add_offsets_(false), is_initialized_(false),
    connection_established_(false), lwr_model_(lwr_model::LWR_URDF),
    lwr_environment_(lwr_environment::LWR_SIMULATION),
    lwr_chain_(), lwr_tree_(), lwr_urdf_model_(),
    linear_root_acc_(lwr_constants::root_acceleration[0],
                     lwr_constants::root_acceleration[1],
                     lwr_constants::root_acceleration[2]),
    angular_root_acc_(lwr_constants::root_acceleration[3],
                      lwr_constants::root_acceleration[4],
                      lwr_constants::root_acceleration[5]),
    root_acc_(linear_root_acc_, angular_root_acc_),
    q_measured_(Eigen::VectorXd::Zero(lwr_constants::NUMBER_OF_JOINTS)),
    qd_measured_(Eigen::VectorXd::Zero(lwr_constants::NUMBER_OF_JOINTS)),
    tau_measured_(Eigen::VectorXd::Zero(lwr_constants::NUMBER_OF_JOINTS)), 
    q_setpoint_(Eigen::VectorXd::Zero(lwr_constants::NUMBER_OF_JOINTS)),
    qd_setpoint_(Eigen::VectorXd::Zero(lwr_constants::NUMBER_OF_JOINTS)),
    tau_setpoint_(Eigen::VectorXd::Zero(lwr_constants::NUMBER_OF_JOINTS)) 
{   

}

//Get Joint Positions
void lwr_mediator::get_joint_positions(KDL::JntArray &joint_positions) 
{
    assert(joint_positions.rows() == lwr_constants::NUMBER_OF_JOINTS);

    // if (lwr_environment_ != lwr_environment::SIMULATION)

    // Assign to the current state the previously passed command 
    joint_positions.data = q_setpoint_;
}

//Set Joint Positions
void lwr_mediator::set_joint_positions(const KDL::JntArray &joint_positions)
{
    assert(joint_positions.rows() == lwr_constants::NUMBER_OF_JOINTS);

    q_setpoint_ = joint_positions.data;

    // if (lwr_environment_ != lwr_environment::SIMULATION)
}

//Get Joint Velocities
void lwr_mediator::get_joint_velocities(KDL::JntArray &joint_velocities)
{
    assert(joint_velocities.rows() == lwr_constants::NUMBER_OF_JOINTS);

    // if (lwr_environment_ != lwr_environment::SIMULATION)
 
    // Assign to the current state the previously passed command  
    joint_velocities.data = qd_setpoint_;
}

//Set Joint Velocities
void lwr_mediator::set_joint_velocities(const KDL::JntArray &joint_velocities)
{
    assert(joint_velocities.rows() == lwr_constants::NUMBER_OF_JOINTS);
   
    qd_setpoint_ = joint_velocities.data;

    // if (lwr_environment_ != lwr_environment::SIMULATION)
}

//Get Joint Torques
void lwr_mediator::get_joint_torques(KDL::JntArray &joint_torques)
{
    assert(joint_torques.rows() == lwr_constants::NUMBER_OF_JOINTS);

    // if (lwr_environment_ != lwr_environment::SIMULATION)

    // Assign to the current state the previously passed command 
    joint_torques.data = tau_setpoint_;
}

//Set Joint Torques
void lwr_mediator::set_joint_torques(const KDL::JntArray &joint_torques) 
{
    assert(joint_torques.rows() == lwr_constants::NUMBER_OF_JOINTS);

    tau_setpoint_ = joint_torques.data;
    
    // if (lwr_environment_ != lwr_environment::SIMULATION)
}

void lwr_mediator::set_joint_command(const KDL::JntArray &joint_positions,
                                     const KDL::JntArray &joint_velocities,
                                     const KDL::JntArray &joint_torques,
                                     const int desired_control_mode)
{
    assert(joint_positions.rows() == lwr_constants::NUMBER_OF_JOINTS);
    assert(joint_velocities.rows() == lwr_constants::NUMBER_OF_JOINTS);
    assert(joint_torques.rows() == lwr_constants::NUMBER_OF_JOINTS);

    if (lwr_environment_ == lwr_environment::LWR_SIMULATION)
    {
        set_joint_torques(joint_torques);
        set_joint_velocities(joint_velocities);
        set_joint_positions(joint_positions);
    }

    else
    {
        switch (desired_control_mode)
        {   
            case control_mode::TORQUE:
                return set_joint_torques(joint_torques);

            case control_mode::VELOCITY:
                return set_joint_velocities(joint_velocities);

            case control_mode::POSITION:
                return set_joint_positions(joint_positions);
        
            default: assert(("Unknown control mode!", false));
        }
    }
}

std::vector<double> lwr_mediator::get_maximum_joint_pos_limits()
{
    return lwr_constants::joint_position_limits_max;
}

std::vector<double> lwr_mediator::get_minimum_joint_pos_limits()
{
    return lwr_constants::joint_position_limits_min;
}

std::vector<double> lwr_mediator::get_joint_position_thresholds()
{
    return lwr_constants::joint_position_thresholds;
}

std::vector<double> lwr_mediator::get_joint_velocity_limits()
{
    return lwr_constants::joint_velocity_limits;
}

std::vector<double> lwr_mediator::get_joint_torque_limits()
{
    return lwr_constants::joint_torque_limits;
}

std::vector<double> lwr_mediator::get_joint_inertia()
{
    return lwr_constants::joint_inertia;
}

std::vector<double> lwr_mediator::get_joint_offsets()
{
    return lwr_constants::joint_offsets;
}

KDL::Twist lwr_mediator::get_root_acceleration()
{
    return root_acc_;
}

KDL::Chain lwr_mediator::get_robot_model() 
{
    return lwr_chain_; 
}

//Extract lwr model from URDF file
int lwr_mediator::get_model_from_urdf()
{
    if (!lwr_urdf_model_.initFile(lwr_constants::urdf_path))
    {
        printf("ERROR: Failed to parse urdf robot model \n");
        return -1;
    }

    //Extract KDL tree from the URDF file
    if (!kdl_parser::treeFromUrdfModel(lwr_urdf_model_, lwr_tree_))
    {
        printf("ERROR: Failed to construct kdl tree \n");
        return -1;
    }

    //Extract KDL chain from KDL tree
    lwr_tree_.getChain(lwr_constants::root_name, 
                       lwr_constants::tooltip_name, 
                       lwr_chain_);
    return 0;
}

//Extract LWR model from KDL parameters
void lwr_mediator::get_kdl_model()
{
    lwr_kdl_model kdl_lwr(lwr_chain_);
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
    int parser_result = 0;
    
    //Extract LWR model from KDL parameters
    if(lwr_model_ == lwr_model::LWR_KDL) get_kdl_model();
    
    //Extract lwr model from the URDF file
    else parser_result = get_model_from_urdf();
    
    if(parser_result != 0) printf("Cannot create the LWR model! \n");
    else
    {
        is_initialized_ = true;
	    printf("LWR initialized successfully! \n");
    } 
}