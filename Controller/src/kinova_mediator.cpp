/*
Author(s): Djordje Vukcevic, Sven Schneider
Description: Mediator component for enabling conversion of data types.

Copyright (c) [2020]

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

#include "kinova_mediator.hpp"
#define IP_ADDRESS "192.168.1.10"
#define PORT 10000
#define PORT_REAL_TIME 10001
#define ACTUATOR_COUNT 7

kinova_mediator::kinova_mediator(): 
    is_initialized_(false), ROBOT_ID_(robot_id::KINOVA_GEN3),
    kinova_model_(kinova_model::URDF),
    kinova_environment_(kinova_environment::SIMULATION),
    add_offsets_(false), connection_established_(false),
    linear_root_acc_(kinova_constants::root_acceleration[0],
                     kinova_constants::root_acceleration[1],
                     kinova_constants::root_acceleration[2]),
    angular_root_acc_(kinova_constants::root_acceleration[3],
                      kinova_constants::root_acceleration[4],
                      kinova_constants::root_acceleration[5]),
    root_acc_(linear_root_acc_, angular_root_acc_)
{
    // Create API error-callback and objects
    auto error_callback = [](Kinova::Api::KError err){ cout << "_________ callback error _________" << err.toString(); };
    this->transport_ = std::make_shared<Kinova::Api::TransportClientTcp>();
    this->transport_real_time_ = std::make_shared<Kinova::Api::TransportClientUdp>();

    this->router_ = std::make_shared<Kinova::Api::RouterClient>(transport_.get(), error_callback);
    this->router_real_time_ = std::make_shared< Kinova::Api::RouterClient>(transport_real_time_.get(), error_callback);
    this->session_manager_ = std::make_shared<Kinova::Api::SessionManager>(router_.get());
    this->session_manager_real_time_ = std::make_shared< Kinova::Api::SessionManager>(router_real_time_.get());
    this->base_ = std::make_shared<Kinova::Api::Base::BaseClient>(router_.get());
    this->base_cyclic_ = std::make_shared< Kinova::Api::BaseCyclic::BaseCyclicClient>(router_real_time_.get());
    this->actuator_config_ = std::make_shared< Kinova::Api::ActuatorConfig::ActuatorConfigClient>(router_.get());
}

kinova_mediator::~kinova_mediator()
{
    deinitialize();
}

// Get Joint Positions
void kinova_mediator::get_joint_positions(KDL::JntArray &joint_positions) 
{
    // Joint position given in deg
    if (kinova_environment_ != kinova_environment::SIMULATION)
    {
        base_feedback_ = base_cyclic_->RefreshFeedback();

        for (int i = 0; i < kinova_constants::NUMBER_OF_JOINTS; i++)
            joint_positions(i) = DEG_TO_RAD(base_feedback_.actuators(i).position());
    }
    else // Assign to the current state the previously passed command 
    {
        for (int i = 0; i < kinova_constants::NUMBER_OF_JOINTS; i++)
            joint_positions(i) = DEG_TO_RAD(base_command_.actuators(i).position());
    }
}

//Set Joint Positions
void kinova_mediator::set_joint_positions(const KDL::JntArray &joint_positions)
{
    // Set actuators in position mode
    control_mode_message_.set_control_mode(Kinova::Api::ActuatorConfig::ControlMode::POSITION);
    for (int actuator_id = 1; actuator_id < ACTUATOR_COUNT + 1; actuator_id++)
        actuator_config_->SetControlMode(control_mode_message_, actuator_id);

    for (int i = 0; i < kinova_constants::NUMBER_OF_JOINTS; i++)
        base_command_.mutable_actuators(i)->set_position(RAD_TO_DEG(joint_positions(i)));

    if (kinova_environment_ != kinova_environment::SIMULATION)
    {
        try
        {
            base_cyclic_->RefreshCommand(base_command_);
        }
        catch (Kinova::Api::KDetailedException& ex)
        {
            std::cout << "Kortex exception: " << ex.what() << std::endl;
            std::cout << "Error sub-code: " << Kinova::Api::SubErrorCodes_Name(Kinova::Api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))) << std::endl;
        }
        catch (std::runtime_error& ex2)
        {
            std::cout << "runtime error: " << ex2.what() << std::endl;
        }
        catch(...)
        {
            std::cout << "Unknown error." << std::endl;
        }
    }
}

// Get Joint Velocities
void kinova_mediator::get_joint_velocities(KDL::JntArray &joint_velocities)
{
    // Joint velocity given in deg/sec
    if (kinova_environment_ != kinova_environment::SIMULATION)
    {
        base_feedback_ = base_cyclic_->RefreshFeedback();

        for (int i = 0; i < kinova_constants::NUMBER_OF_JOINTS; i++)
            joint_velocities(i) = DEG_TO_RAD(base_feedback_.actuators(i).velocity());
    }
    else // Assign to the current state the previously passed command 
    {
        for (int i = 0; i < kinova_constants::NUMBER_OF_JOINTS; i++)
            joint_velocities(i) = DEG_TO_RAD(base_command_.actuators(i).velocity());
    }
}

// Set Joint Velocities
void kinova_mediator::set_joint_velocities(const KDL::JntArray &joint_velocities)
{   
    // Set actuators in velocity mode
    control_mode_message_.set_control_mode(Kinova::Api::ActuatorConfig::ControlMode::VELOCITY);
    for (int actuator_id = 1; actuator_id < ACTUATOR_COUNT + 1; actuator_id++)
        actuator_config_->SetControlMode(control_mode_message_, actuator_id);

    for (int i = 0; i < kinova_constants::NUMBER_OF_JOINTS; i++)
    {
        base_command_.mutable_actuators(i)->set_position(base_feedback_.actuators(i).position());
        base_command_.mutable_actuators(i)->set_velocity(RAD_TO_DEG(joint_velocities(i)));
    }

    if (kinova_environment_ != kinova_environment::SIMULATION)
    {
        try
        {
            base_cyclic_->RefreshCommand(base_command_);
        }
        catch (Kinova::Api::KDetailedException& ex)
        {
            std::cout << "Kortex exception: " << ex.what() << std::endl;
            std::cout << "Error sub-code: " << Kinova::Api::SubErrorCodes_Name(Kinova::Api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))) << std::endl;
        }
        catch (std::runtime_error& ex2)
        {
            std::cout << "runtime error: " << ex2.what() << std::endl;
        }
        catch(...)
        {
            std::cout << "Unknown error." << std::endl;
        }
    }
}

// Get Joint Torques
void kinova_mediator::get_joint_torques(KDL::JntArray &joint_torques)
{
    // Joint torque given in Newton * meters
    if (kinova_environment_ != kinova_environment::SIMULATION)
    {
        base_feedback_ = base_cyclic_->RefreshFeedback();

        for (int i = 0; i < kinova_constants::NUMBER_OF_JOINTS; i++)
            joint_torques(i) = base_feedback_.actuators(i).torque();
    }
    else // Assign to the current state the previously passed command 
    {
        for (int i = 0; i < kinova_constants::NUMBER_OF_JOINTS; i++)
            joint_torques(i) = base_command_.actuators(i).torque_joint();
    }
}

// Set Joint Torques
void kinova_mediator::set_joint_torques(const KDL::JntArray &joint_torques) 
{
    // Set actuators in torque mode
    control_mode_message_.set_control_mode(Kinova::Api::ActuatorConfig::ControlMode::TORQUE);
    for (int actuator_id = 1; actuator_id < ACTUATOR_COUNT + 1; actuator_id++)
        actuator_config_->SetControlMode(control_mode_message_, actuator_id);

    for (int i = 0; i < kinova_constants::NUMBER_OF_JOINTS; i++)
    {
        base_command_.mutable_actuators(i)->set_position(base_feedback_.actuators(i).position());
        base_command_.mutable_actuators(i)->set_torque_joint(joint_torques(i));
    }

    if (kinova_environment_ != kinova_environment::SIMULATION)
    {
        try
        {
            base_cyclic_->RefreshCommand(base_command_);
        }
        catch (Kinova::Api::KDetailedException& ex)
        {
            std::cout << "Kortex exception: " << ex.what() << std::endl;
            std::cout << "Error sub-code: " << Kinova::Api::SubErrorCodes_Name(Kinova::Api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))) << std::endl;
        }
        catch (std::runtime_error& ex2)
        {
            std::cout << "runtime error: " << ex2.what() << std::endl;
        }
        catch(...)
        {
            std::cout << "Unknown error." << std::endl;
        }
    }
}

void kinova_mediator::set_joint_command(const KDL::JntArray &joint_positions,
                                        const KDL::JntArray &joint_velocities,
                                        const KDL::JntArray &joint_torques,
                                        const int desired_control_mode)
{
    assert(joint_positions.rows()  == kinova_constants::NUMBER_OF_JOINTS);
    assert(joint_velocities.rows() == kinova_constants::NUMBER_OF_JOINTS);
    assert(joint_torques.rows()    == kinova_constants::NUMBER_OF_JOINTS);

    // Incrementing identifier ensures actuators can reject out of time frames
    // Buffer?
    base_command_.set_frame_id(base_command_.frame_id() + 1);
    if (base_command_.frame_id() > 65535) base_command_.set_frame_id(0);

    for (int i = 0; i < kinova_constants::NUMBER_OF_JOINTS; i++)
        base_command_.mutable_actuators(i)->set_command_id(base_command_.frame_id());

    if (kinova_environment_ == kinova_environment::SIMULATION)
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
                break;
        }
    }
}

bool kinova_mediator::robot_stopped()
{
    // Check if velocity control mode is active
    if (control_mode_message_.control_mode() != Kinova::Api::ActuatorConfig::ControlMode::VELOCITY) return false;

    base_feedback_ = base_cyclic_->RefreshFeedback();
    for (int i = 0; i < kinova_constants::NUMBER_OF_JOINTS; i++)
    {
        // Check if velocity setpoint is zero
        if ((base_feedback_.actuators(i).velocity() != 0.0) || \
            !std::isfinite(base_feedback_.actuators(i).velocity())) return false;
    }
    return true;
}

// Set Zero Joint Velocities and wait until robot has stopped completely
void kinova_mediator::stop_robot_motion()
{
    // Incrementing identifier ensures actuators can reject out of time frames
    // Buffer?
    base_command_.set_frame_id(base_command_.frame_id() + 1);
    if (base_command_.frame_id() > 65535) base_command_.set_frame_id(0);

    for (int i = 0; i < kinova_constants::NUMBER_OF_JOINTS; i++)
        base_command_.mutable_actuators(i)->set_command_id(base_command_.frame_id());

    // Set actuators in velocity mode
    control_mode_message_.set_control_mode(Kinova::Api::ActuatorConfig::ControlMode::VELOCITY);
    for (int actuator_id = 1; actuator_id < ACTUATOR_COUNT + 1; actuator_id++)
        actuator_config_->SetControlMode(control_mode_message_, actuator_id);

    // Send the zero velocity commands to motors
    for (int i = 0; i < kinova_constants::NUMBER_OF_JOINTS; i++)
    {
        base_command_.mutable_actuators(i)->set_position(base_feedback_.actuators(i).position());
        base_command_.mutable_actuators(i)->set_velocity(0.0);
    }

    if (kinova_environment_ != kinova_environment::SIMULATION)
    {
        try
        {
            base_cyclic_->RefreshCommand(base_command_);
            
            // Monitor robot state until robot has stopped completely
            bool wait_for_driver = true;
            while (wait_for_driver)
            {
                if (robot_stopped()) wait_for_driver = false;
            }
        }
        catch (Kinova::Api::KDetailedException& ex)
        {
            std::cout << "Kortex exception: " << ex.what() << std::endl;
            std::cout << "Error sub-code: " << Kinova::Api::SubErrorCodes_Name(Kinova::Api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))) << std::endl;
        }
        catch (std::runtime_error& ex2)
        {
            std::cout << "runtime error: " << ex2.what() << std::endl;
        }
        catch(...)
        {
            std::cout << "Unknown error." << std::endl;
        }
    }
}

std::vector<double> kinova_mediator::get_maximum_joint_pos_limits()
{
    return kinova_constants::joint_position_limits_max;
}

std::vector<double> kinova_mediator::get_minimum_joint_pos_limits()
{
    return kinova_constants::joint_position_limits_min;
}

std::vector<double> kinova_mediator::get_joint_position_thresholds()
{
    return kinova_constants::joint_position_thresholds;
}

std::vector<double> kinova_mediator::get_joint_velocity_limits()
{
    return kinova_constants::joint_velocity_limits;
}

std::vector<double> kinova_mediator::get_joint_torque_limits()
{
    return kinova_constants::joint_torque_limits;
}

std::vector<double> kinova_mediator::get_joint_inertia()
{
    return kinova_constants::joint_inertia;
}

std::vector<double> kinova_mediator::get_joint_offsets()
{
    return kinova_constants::joint_offsets;
}

KDL::Twist kinova_mediator::get_root_acceleration()
{
    return root_acc_;
}

KDL::Chain kinova_mediator::get_robot_model() 
{
    return kinova_chain_; 
}

//Extract youBot model from URDF file
int kinova_mediator::get_model_from_urdf()
{
    if (!kinova_urdf_model_.initFile(kinova_constants::urdf_path))
    {
        printf("ERROR: Failed to parse urdf robot model \n");
        return -1;
    }

    //Extract KDL tree from the URDF file
    if (!kdl_parser::treeFromUrdfModel(kinova_urdf_model_, kinova_tree_))
    {
        printf("ERROR: Failed to construct kdl tree \n");
        return -1;
    }

    //Extract KDL chain from KDL tree
    kinova_tree_.getChain(kinova_constants::root_name, 
                          kinova_constants::tooltip_name, 
                          kinova_chain_);
    return 0;
}

int kinova_mediator::get_robot_ID()
{
    return ROBOT_ID_;
}

bool kinova_mediator::is_initialized()
{
    return is_initialized_;
}

// Initialize variables and calibrate the manipulator: 
void kinova_mediator::initialize(const int robot_model,
                                 const int robot_environment,
                                 const bool gravity_compensated)
{
    kinova_model_       = robot_model;
    kinova_environment_ = robot_environment;
    kinova_chain_       = KDL::Chain();

    // Reset Flags
    is_initialized_   = false;
    add_offsets_      = false;
    int parser_result = 0;

    //Extract Kinova model from the URDF file
    parser_result = get_model_from_urdf();

    if (kinova_environment_ != kinova_environment::SIMULATION && !connection_established_)
    {
        // Create API objects
        transport_->connect(IP_ADDRESS, PORT);
        transport_real_time_->connect(IP_ADDRESS, PORT_REAL_TIME);

        // Set session data connection information
        auto create_session_info = Kinova::Api::Session::CreateSessionInfo();
        create_session_info.set_username("admin");
        create_session_info.set_password("admin");
        create_session_info.set_session_inactivity_timeout(1000);   // (milliseconds)
        create_session_info.set_connection_inactivity_timeout(100); // (milliseconds)

        // Session manager service wrapper
        session_manager_->CreateSession(create_session_info);
        session_manager_real_time_->CreateSession(create_session_info);
        std::cout << "Kinova sessions created" << std::endl;

        // Clearing faults
        try
        {
            base_->ClearFaults();
        }
        catch(...)
        {
            std::cout << "Unable to clear robot faults" << std::endl;
            return;
        }

        try
        {
            // Set the robot in low-level servoing mode
            servoing_mode_.set_servoing_mode(Kinova::Api::Base::ServoingMode::LOW_LEVEL_SERVOING);
            base_->SetServoingMode(servoing_mode_);
            base_feedback_ = base_cyclic_->RefreshFeedback();

            // Initialize each actuator to their current position
            for (int i = 0; i < ACTUATOR_COUNT; i++)
                base_command_.add_actuators()->set_position(base_feedback_.actuators(i).position());

            // Send a first command (time frame)... position command in this case
            base_feedback_ = base_cyclic_->Refresh(base_command_);
        }
        catch (Kinova::Api::KDetailedException& ex)
        {
            std::cout << "API error: " << ex.what() << std::endl;
            return;
        }
        catch (std::runtime_error& ex2)
        {
            std::cout << "Error: " << ex2.what() << std::endl;
            return;
        }

        connection_established_ = true;
    }

    if (parser_result != 0)  printf("Cannot create Kinova model! \n");
    else
    {
        is_initialized_ = true;
        printf("Kinova initialized successfully! \n");
    } 
}

void kinova_mediator::deinitialize()
{
    // Close API session
    session_manager_->CloseSession();
    session_manager_real_time_->CloseSession();

    // Deactivate the router and cleanly disconnect from the transport object
    router_->SetActivationStatus(false);
    transport_->disconnect();
    router_real_time_->SetActivationStatus(false);
    transport_real_time_->disconnect();

    printf("Robot deinitialized \n");

    // Destroy the API
//     delete base_;
//     delete base_cyclic_;
//     delete actuator_config_;
//     delete session_manager_;
//     delete session_manager_real_time_;
//     delete router_;
//     delete router_real_time_;
//     delete transport_;
//     delete transport_real_time_;
}
