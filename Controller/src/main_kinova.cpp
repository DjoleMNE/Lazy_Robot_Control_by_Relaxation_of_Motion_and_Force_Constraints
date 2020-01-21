#include <iostream>
#include <string>
#include <vector>
#include <math.h>

#include <KDetailedException.h>

#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <ActuatorConfigClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>

#include <RouterClient.h>
#include <TransportClientUdp.h>
#include <TransportClientTcp.h>

#include <google/protobuf/util/json_util.h>
#include <unistd.h>
#include <time.h>

#define IP_ADDRESS "192.168.1.10"

#define PORT 10000
#define PORT_REAL_TIME 10001

#define ACTUATOR_COUNT 7
#define DURATION 30 // Network timeout (seconds)

float time_duration = 30.0; // Duration of the example (seconds)

/*****************************
 * Example related function *
 *****************************/
int64_t GetTickUs()
{
    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);

    return (start.tv_sec * 1000000LLU) + (start.tv_nsec / 1000);
}

/**************************
 * Example core functions *
 **************************/
void example_move_to_home_position(Kinova::Api::Base::BaseClient* base)
{
    // Make sure the arm is in Single (High) Level Servoing before executing an Action
    auto servoingMode = Kinova::Api::Base::ServoingModeInformation();
    servoingMode.set_servoing_mode(Kinova::Api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Move arm to ready position
    std::cout << "Moving the arm to a safe position" << std::endl;
    auto action_type = Kinova::Api::Base::RequestedActionType();
    action_type.set_action_type(Kinova::Api::Base::REACH_JOINT_ANGLES);
    auto action_list = base->ReadAllActions(action_type);
    auto action_handle = Kinova::Api::Base::ActionHandle();
    action_handle.set_identifier(0);
    for (auto action : action_list.action_list()) 
    {
        if (action.name() == "Home") 
        {
            action_handle = action.handle();
        }
    }

    if (action_handle.identifier() == 0) 
    {
        std::cout << "Can't reach safe position, exiting" << std::endl;
    } 
    else 
    {
        base->ExecuteActionFromReference(action_handle);
        std::this_thread::sleep_for(std::chrono::milliseconds(12000)); // Leave time to action to finish
    }
}

bool example_cyclic_torque_control(Kinova::Api::Base::BaseClient* base, Kinova::Api::BaseCyclic::BaseCyclicClient* base_cyclic, Kinova::Api::ActuatorConfig::ActuatorConfigClient* actuator_config)
{
    bool return_status = true;

    // Clearing faults
    try
    {
        base->ClearFaults();
    }
    catch(...)
    {
        std::cout << "Unable to clear robot faults" << std::endl;
        return false;
    }

    
    Kinova::Api::BaseCyclic::Feedback base_feedback;
    Kinova::Api::BaseCyclic::Command  base_command;
    // std::vector<float> commands;

    auto servoing_mode = Kinova::Api::Base::ServoingModeInformation();

    int timer_count = 0;
    int64_t now = 0;
    int64_t last = 0;

    std::cout << "Initializing the arm for torque control example" << std::endl;
    try
    {
        // Set the base in low-level servoing mode
        servoing_mode.set_servoing_mode(Kinova::Api::Base::ServoingMode::LOW_LEVEL_SERVOING);
        base->SetServoingMode(servoing_mode);
        base_feedback = base_cyclic->RefreshFeedback();

        // Initialize each actuator to their current position
        for (int i = 0; i < ACTUATOR_COUNT; i++)
        {
            // commands.push_back(base_feedback.actuators(i).position());

            // Save the current actuator position, to avoid a following error
            base_command.add_actuators()->set_position(base_feedback.actuators(i).position());
        }

        // Send a first command (time frame)... position command in this case
        base_feedback = base_cyclic->Refresh(base_command);
        
        // Set only the first actuator in torque mode... now that the command is equal to measured
        auto control_mode_message = Kinova::Api::ActuatorConfig::ControlModeInformation();
        control_mode_message.set_control_mode(Kinova::Api::ActuatorConfig::ControlMode::TORQUE);

        // Are other actuators by default in POSITION control mode?
        int first_actuator_device_id = 1;
        actuator_config->SetControlMode(control_mode_message, first_actuator_device_id);

        // Initial delta between first and last actuator
        float init_delta_position = base_feedback.actuators(0).position() - base_feedback.actuators(6).position();

        // Initial first and last actuator torques; avoids unexpected movement due to torque offsets
        float init_last_torque = base_feedback.actuators(6).torque();
        float init_first_torque = -base_feedback.actuators(0).torque(); //Torque measure is reversed compared to actuator direction
        float torque_amplification = 2.0;

        std::cout << "Running torque control example for " << time_duration << " seconds" << std::endl;

        // Real-time loop
        while (timer_count < (time_duration * 1000))
        {
            now = GetTickUs();

            // Ensure constant loop frequency
            if (now - last > 1000)
            {
                // Position command to first actuator is set to the measured one... in order to avoid a following error to trigger
                // Bonus: When doing this, instead of disabling the following error, if communication is lost and first
                //        actuator continues to move under torque command, resulting position error with command will
                //        trigger a following error and switch back the actuator in position command to hold its position
                //        In simple words, prepare (record) the position command for every case, if an error occurs?
                base_command.mutable_actuators(0)->set_position(base_feedback.actuators(0).position());

                // First actuator torque command is set to last actuator torque measure times an amplification
                base_command.mutable_actuators(0)->set_torque_joint(init_first_torque + (torque_amplification * (base_feedback.actuators(6).torque() - init_last_torque)));

                // First actuator position is sent as a command to last actuator
                base_command.mutable_actuators(6)->set_position(base_feedback.actuators(0).position() - init_delta_position);

                // Incrementing identifier ensures actuators can reject out of time frames
                base_command.set_frame_id(base_command.frame_id() + 1);
                if (base_command.frame_id() > 65535)
                    base_command.set_frame_id(0);

                for (int idx = 0; idx < ACTUATOR_COUNT; idx++)
                {
                    base_command.mutable_actuators(idx)->set_command_id(base_command.frame_id());
                }

                try
                {
                    base_feedback = base_cyclic->Refresh(base_command, 0);
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
                
                timer_count++;
                last = GetTickUs();
            }
        }

        std::cout << "Torque control example completed" << std::endl;

        // Set first actuator back in position 
        control_mode_message.set_control_mode(Kinova::Api::ActuatorConfig::ControlMode::POSITION);
        actuator_config->SetControlMode(control_mode_message, first_actuator_device_id);

        std::cout << "Torque control example clean exit" << std::endl;

    }
    catch (Kinova::Api::KDetailedException& ex)
    {
        std::cout << "API error: " << ex.what() << std::endl;
        return_status = false;
    }
    catch (std::runtime_error& ex2)
    {
        std::cout << "Error: " << ex2.what() << std::endl;
        return_status = false;
    }
    
    // Set the servoing mode back to Single Level
    servoing_mode.set_servoing_mode(Kinova::Api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoing_mode);

    // Wait for a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    return return_status;
}

int main(int argc, char **argv)
{
    // Create API objects
    auto error_callback = [](Kinova::Api::KError err){ cout << "_________ callback error _________" << err.toString(); };
    
    std::cout << "Creating transport objects" << std::endl;
    auto transport = new Kinova::Api::TransportClientTcp();
    auto router = new Kinova::Api::RouterClient(transport, error_callback);
    transport->connect(IP_ADDRESS, PORT);

    std::cout << "Creating transport real time objects" << std::endl;
    auto transport_real_time = new Kinova::Api::TransportClientUdp();
    auto router_real_time = new Kinova::Api::RouterClient(transport_real_time, error_callback);
    transport_real_time->connect(IP_ADDRESS, PORT_REAL_TIME);

    // Set session data connection information
    auto create_session_info = Kinova::Api::Session::CreateSessionInfo();
    create_session_info.set_username("admin");
    create_session_info.set_password("kinova1_area4251");
    create_session_info.set_session_inactivity_timeout(1000);   // (milliseconds)
    create_session_info.set_connection_inactivity_timeout(200); // (milliseconds)

    // Session manager service wrapper
    std::cout << "Creating sessions for communication" << std::endl;
    auto session_manager = new Kinova::Api::SessionManager(router);
    session_manager->CreateSession(create_session_info);
    auto session_manager_real_time = new Kinova::Api::SessionManager(router_real_time);
    session_manager_real_time->CreateSession(create_session_info);
    std::cout << "Sessions created" << std::endl;

    // Create services
    auto base = new Kinova::Api::Base::BaseClient(router);
    auto base_cyclic = new Kinova::Api::BaseCyclic::BaseCyclicClient(router_real_time);
    auto actuator_config = new Kinova::Api::ActuatorConfig::ActuatorConfigClient(router);

    // Example core
    example_move_to_home_position(base);
    auto isOk = example_cyclic_torque_control(base, base_cyclic, actuator_config);
    if (!isOk)
    {
        std::cout << "There has been an unexpected error in example_cyclic_torque_control() function." << endl;;
    }

    // Close API session
    session_manager->CloseSession();
    session_manager_real_time->CloseSession();

    // Deactivate the router and cleanly disconnect from the transport object
    router->SetActivationStatus(false);
    transport->disconnect();
    router_real_time->SetActivationStatus(false);
    transport_real_time->disconnect();

    // Destroy the API
    delete base;
    delete base_cyclic;
    delete actuator_config;
    delete session_manager;
    delete session_manager_real_time;
    delete router;
    delete router_real_time;
    delete transport;
    delete transport_real_time;
}
