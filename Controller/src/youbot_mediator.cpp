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
#include "youbot_mediator.hpp"

youbot_mediator::youbot_mediator(): 
    is_initialized_(false), ROBOT_ID_(robot_id::YOUBOT),
    youbot_model_(youbot_model::URDF),
    youbot_environment_(youbot_environment::SIMULATION),
    add_offsets_(false), connection_established_(false),
    youbot_arm_(nullptr), yb_chain_(), yb_tree_(), yb_urdf_model_(),
    linear_root_acc_(youbot_constants::root_acceleration[0],
                     youbot_constants::root_acceleration[1],
                     youbot_constants::root_acceleration[2]),
    angular_root_acc_(youbot_constants::root_acceleration[3],
                      youbot_constants::root_acceleration[4],
                      youbot_constants::root_acceleration[5]),
    root_acc_(linear_root_acc_, angular_root_acc_) 
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

// Update robot state: measured positions, velocities, torques and measured / estimated external forces on end-effector
void youbot_mediator::get_robot_state(KDL::JntArray &joint_positions,
                                      KDL::JntArray &joint_velocities,
                                      KDL::JntArray &joint_torques,
                                      KDL::Wrench &end_effector_wrench)
{
    get_joint_positions(joint_positions);
    get_joint_velocities(joint_velocities);
    get_joint_torques(joint_torques);
    get_end_effector_wrench(end_effector_wrench);
}

// Update joint space state: measured positions, velocities and torques
void youbot_mediator::get_joint_state(KDL::JntArray &joint_positions,
                                      KDL::JntArray &joint_velocities,
                                      KDL::JntArray &joint_torques)
{
    get_joint_positions(joint_positions);
    get_joint_velocities(joint_velocities);
    get_joint_torques(joint_torques);
}

// Get Joint Positions
void youbot_mediator::get_joint_positions(KDL::JntArray &joint_positions) 
{
    if (youbot_environment_ != youbot_environment::SIMULATION)
    {
	    youbot_arm_->getJointData(q_measured_);

        for (int i = 0; i < youbot_constants::NUMBER_OF_JOINTS; i++) 
        {
            joint_positions(i) = q_measured_[i].angle.value();

            // Custom model's home state is not folded - it is candle
            // Check with Sven if the last joint value should be inverted here
            // similary like in the case of getting joint velocities
            // Check JP's code and report
            if (add_offsets_) joint_positions(i) = joint_positions(i) + youbot_constants::joint_offsets[i];
        }
    }
    else // Assign to the current state the previously passed command 
    {
        for (int i = 0; i < youbot_constants::NUMBER_OF_JOINTS; i++)
            joint_positions(i) = q_setpoint_[i].angle.value();
    }
}

//Set Joint Positions
int youbot_mediator::set_joint_positions(const KDL::JntArray &joint_positions)
{
    for (int i = 0; i < youbot_constants::NUMBER_OF_JOINTS; i++)
    {
        if (add_offsets_) q_setpoint_[i].angle = (joint_positions(i) - youbot_constants::joint_offsets[i]) * radian;
        else q_setpoint_[i].angle = joint_positions(i) * radian;
    }

    if (youbot_environment_ != youbot_environment::SIMULATION) youbot_arm_->setJointData(q_setpoint_);
    return 0;
}

// Get Joint Velocities
void youbot_mediator::get_joint_velocities(KDL::JntArray &joint_velocities)
{
    if (youbot_environment_ != youbot_environment::SIMULATION)
    {
        youbot_arm_->getJointData(qd_measured_);

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

// Set Joint Velocities
int youbot_mediator::set_joint_velocities(const KDL::JntArray &joint_velocities)
{   
    for (int i = 0; i < youbot_constants::NUMBER_OF_JOINTS; i++)
        qd_setpoint_[i].angularVelocity = joint_velocities(i) * radian_per_second;

    // if (add_offsets_) qd_setpoint_[4].angularVelocity = 0.0 * radian_per_second;
    if (add_offsets_) qd_setpoint_[4].angularVelocity = -1 * joint_velocities(4) * radian_per_second;
    if (youbot_environment_ != youbot_environment::SIMULATION) youbot_arm_->setJointData(qd_setpoint_);
    return 0;
}

// Get Joint Torques
void youbot_mediator::get_joint_torques(KDL::JntArray &joint_torques)
{
    if (youbot_environment_ != youbot_environment::SIMULATION)
    {
        youbot_arm_->getJointData(tau_measured_);
        
        for (int i = 0; i < youbot_constants::NUMBER_OF_JOINTS; i++)
            joint_torques(i) = tau_measured_[i].torque.value();
        
        if (add_offsets_) joint_torques(4) = -1 * joint_torques(4);
    }
    else // Assign to the current state, the previously passed command 
    {
        for (int i = 0; i < youbot_constants::NUMBER_OF_JOINTS; i++)
            joint_torques(i) = tau_setpoint_[i].torque.value();
    }
}

// Set Joint Torques
int youbot_mediator::set_joint_torques(const KDL::JntArray &joint_torques) 
{
    for (int i = 0; i < youbot_constants::NUMBER_OF_JOINTS; i++)
        tau_setpoint_[i].torque = joint_torques(i) * newton_meter;
    
    // if(add_offsets_) tau_setpoint_[4].torque = 0.0 * newton_meter;
    if (add_offsets_) tau_setpoint_[4].torque = -1 * joint_torques(4) * newton_meter;
    if (youbot_environment_ != youbot_environment::SIMULATION) youbot_arm_->setJointData(tau_setpoint_);
    return 0;
}

// Get measured / estimated external forces acting on the end-effector
void youbot_mediator::get_end_effector_wrench(KDL::Wrench &end_effector_wrench)
{
    // Linear forces given in Newton, angular in Newton * meters
    KDL::SetToZero(end_effector_wrench);   
}

int youbot_mediator::set_joint_command(const KDL::JntArray &joint_positions,
                                        const KDL::JntArray &joint_velocities,
                                        const KDL::JntArray &joint_torques,
                                        const int desired_control_mode)
{
    assert(joint_positions.rows()  == youbot_constants::NUMBER_OF_JOINTS);
    assert(joint_velocities.rows() == youbot_constants::NUMBER_OF_JOINTS);
    assert(joint_torques.rows()    == youbot_constants::NUMBER_OF_JOINTS);

    if (youbot_environment_ == youbot_environment::SIMULATION)
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
        
            default: 
                assert(("Unknown control mode!", false));
                return -1;
        }
    }
    return 0;
}

bool youbot_mediator::get_bit(unsigned int flag, const int position)
{
    return (flag >> position) & 0x1;
}

bool youbot_mediator::robot_stopped()
{
    unsigned int status_flag = 0;

    for (int i = 0; i < youbot_constants::NUMBER_OF_JOINTS; i++)
    {
        // Check if velocity control mode is active
        youbot_arm_->getArmJoint(i + 1).getStatus(status_flag);
        if (!get_bit(status_flag, 9)) return false;

        // Check if velocity setpoint is zero
        youbot_arm_->getArmJoint(i + 1).getData(qd_setpoint_[i]);
        if ((qd_setpoint_[i].angularVelocity.value() != 0.0) || \
            !std::isfinite(qd_setpoint_[i].angularVelocity.value())) return false;
    }
    return true;
}

// Set Zero Joint Velocities and wait until robot has stopped completely
int youbot_mediator::stop_robot_motion()
{
    // Send the zero velocity commands to motors
    for (int i = 0; i < youbot_constants::NUMBER_OF_JOINTS; i++)
        qd_setpoint_[i].angularVelocity = 0.0 * radian_per_second;

    if (youbot_environment_ != youbot_environment::SIMULATION)
    {
        youbot_arm_->setJointData(qd_setpoint_);

        // Monitor robot state until robot has stopped completely
        bool wait_for_driver = true;
        while (wait_for_driver)
        {
            if (robot_stopped()) wait_for_driver = false;
        }
    }
    return 0;
}

std::vector<double> youbot_mediator::get_maximum_joint_pos_limits()
{
    if (youbot_model_ == youbot_model::YB_STORE) return youbot_constants::joint_position_limits_max_1;
    else 
    {
        if (youbot_environment_ != youbot_environment::SIMULATION) return youbot_constants::joint_position_limits_max_2;
        return youbot_constants::joint_position_limits_max_2_sim;
    }
}

std::vector<double> youbot_mediator::get_minimum_joint_pos_limits()
{
    if (youbot_model_ == youbot_model::YB_STORE) return youbot_constants::joint_position_limits_min_1;
    else 
    {
        if (youbot_environment_ != youbot_environment::SIMULATION) return youbot_constants::joint_position_limits_min_2;
        return youbot_constants::joint_position_limits_min_2_sim;
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

std::vector<double> youbot_mediator::get_joint_acceleration_limits()
{
    assert(youbot_constants::NUMBER_OF_JOINTS == youbot_constants::joint_acceleration_limits.size());
    return youbot_constants::joint_acceleration_limits;
}

std::vector<double> youbot_mediator::get_joint_torque_limits()
{
    return youbot_constants::joint_torque_limits;
}

std::vector<double> youbot_mediator::get_joint_stopping_torque_limits()
{
    assert(youbot_constants::NUMBER_OF_JOINTS == youbot_constants::joint_stopping_torque_limits.size());
    return youbot_constants::joint_stopping_torque_limits;
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
    return yb_chain_; 
}

// Above main chain is prepared for vereshchagin (nj == ns) but this full contains additional segments
KDL::Chain youbot_mediator::get_full_robot_model() 
{
    //To be implemented
    return yb_chain_; 
}

//Extract youBot model from URDF file
int youbot_mediator::get_model_from_urdf()
{
    if (!yb_urdf_model_.initFile(youbot_constants::urdf_path))
    {
        printf("ERROR: Failed to parse urdf robot model \n");
        return -1;
    }

    //Extract KDL tree from the URDF file
    if (!kdl_parser::treeFromUrdfModel(yb_urdf_model_, yb_tree_))
    {
        printf("ERROR: Failed to construct kdl tree \n");
        return -1;
    }

    //Extract KDL chain from KDL tree
    yb_tree_.getChain(youbot_constants::root_name, 
                      youbot_constants::tooltip_name, 
                      yb_chain_);
    return 0;
}

int youbot_mediator::get_robot_ID()
{
    return ROBOT_ID_;
}

bool youbot_mediator::is_initialized()
{
    return is_initialized_;
}

// Initialize variables and calibrate the manipulator: 
void youbot_mediator::initialize(const int robot_model,
                                 const int robot_environment,
                                 const int id)
{
    youbot_model_       = robot_model;
    youbot_environment_ = robot_environment;
    yb_chain_           = KDL::Chain();

    // Reset Flags
    is_initialized_   = false;
    add_offsets_      = false;
    int parser_result = 0;

    if (youbot_model_ == youbot_model::YB_STORE)
    {
        //Extract model from custom file 
        youbot_custom_model yb_store_model(yb_chain_);
        
        // Add offsets to match real robot and the youBot store model
        if (youbot_environment_ != youbot_environment::SIMULATION) add_offsets_ = true;
    }
    //Extract youBot model from the URDF file
    else parser_result = get_model_from_urdf();
    
    if (youbot_environment_ != youbot_environment::SIMULATION && !connection_established_)
    {
        this->youbot_arm_ = std::make_shared<youbot::YouBotManipulator>("youbot-manipulator", 
                                                                        youbot_constants::config_path);
        // Commutate with the joints
        youbot_arm_->doJointCommutation();
        // Calibrate youBot arm
        youbot_arm_->calibrateManipulator();
        connection_established_ = true;
    }
    
    if (parser_result != 0)  printf("Cannot create youBot model! \n");
    else
    {
        is_initialized_ = true;
	    printf("youBot initialized successfully! \n");
    } 
}