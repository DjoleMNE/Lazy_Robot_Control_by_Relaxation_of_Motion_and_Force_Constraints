#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <stdlib.h> /* abs */
#include <chrono>
#include <time.h>
#include <unistd.h>
#include <KDetailedException.h>

#include <kdl/kdl.hpp>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <abag.hpp>

#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <ActuatorConfigClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>

#include <RouterClient.h>
#include <TransportClientUdp.h>
#include <TransportClientTcp.h>
#include <google/protobuf/text_format.h>
#include <google/protobuf/util/json_util.h>
const int SECOND = 1000000;

namespace k_api = Kinova::Api;

#define IP_ADDRESS "192.168.1.10"

#define PORT 10000
#define PORT_REAL_TIME 10001

#define ACTUATOR_COUNT 7
#define DEG_TO_RAD(x) (x) * 3.14159265358979323846 / 180.0
#define RAD_TO_DEG(x) (x) * 180.0 / 3.14159265358979323846
float TIME_DURATION = 30.0f; // Duration of the example (seconds)

// Maximum allowed waiting time during actions
constexpr auto TIMEOUT_PROMISE_DURATION = std::chrono::seconds{20};
std::chrono::steady_clock::time_point loop_start_time;
std::chrono::duration <double, std::micro> loop_interval{};

/*****************************
 * Example related function *
 *****************************/
int64_t GetTickUs()
{
    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);

    return (start.tv_sec * 1000000LLU) + (start.tv_nsec / 1000);
}

//Make sure that the control loop runs exactly with the specified frequency
int enforce_loop_frequency(const int dt)
{
    loop_interval = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - loop_start_time);

    if (loop_interval < std::chrono::microseconds(dt)) // Loop is sufficiently fast
    {
        while (loop_interval < std::chrono::microseconds(dt - 1))
            loop_interval = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - loop_start_time);

        return 0;
    }
    else return -1; //Loop is too slow
}

// Create an event listener that will set the promise action event to the exit value
// Will set promise to either END or ABORT
// Use finish_promise.get_future.get() to wait and get the value
std::function<void(k_api::Base::ActionNotification)> 
    create_event_listener_by_promise(std::promise<k_api::Base::ActionEvent>& finish_promise)
{
    return [&finish_promise] (k_api::Base::ActionNotification notification)
    {
        const auto action_event = notification.action_event();
        switch(action_event)
        {
        case k_api::Base::ActionEvent::ACTION_END:
        case k_api::Base::ActionEvent::ACTION_ABORT:
            finish_promise.set_value(action_event);
            break;
        default:
            break;
        }
    };
}

/**************************
 * Example core functions *
 **************************/
void example_move_to_home_position(k_api::Base::BaseClient* base)
{
    // Make sure the arm is in Single Level Servoing before executing an Action
    auto servoingMode = k_api::Base::ServoingModeInformation();
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Move arm to ready position
    std::cout << "Moving the arm to a safe position" << std::endl;
    auto action_type = k_api::Base::RequestedActionType();
    action_type.set_action_type(k_api::Base::REACH_JOINT_ANGLES);
    auto action_list = base->ReadAllActions(action_type);
    auto action_handle = k_api::Base::ActionHandle();
    action_handle.set_identifier(0);

    for (auto action : action_list.action_list()) 
    {
        if (action.name() == "Home") 
        {
            action_handle = action.handle();
        }
    }

    if (action_handle.identifier() == 0) std::cout << "Can't reach safe position, exiting" << std::endl;
    else
    {
        // Connect to notification action topic
        std::promise<k_api::Base::ActionEvent> finish_promise;
        auto finish_future = finish_promise.get_future();
        auto promise_notification_handle = base->OnNotificationActionTopic(
            create_event_listener_by_promise(finish_promise),
            k_api::Common::NotificationOptions()
        );

        // Execute action
        base->ExecuteActionFromReference(action_handle);

        // Wait for future value from promise
        const auto status = finish_future.wait_for(TIMEOUT_PROMISE_DURATION);
        base->Unsubscribe(promise_notification_handle);

        if(status != std::future_status::ready)
        {
            std::cout << "Timeout on action notification wait" << std::endl;
        }
        const auto promise_event = finish_future.get();
    }
    base->Stop();
    // std::this_thread::sleep_for(std::chrono::milliseconds(1500));
}

bool example_cyclic_torque_control(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic, k_api::ActuatorConfig::ActuatorConfigClient* actuator_config)
{
    const int RATE_HZ = 999; // Hz
    const int DT_MICRO = SECOND / RATE_HZ;
    const double DT_SEC = 1.0 / static_cast<double>(RATE_HZ);

    double total_time_sec = 0.0;
    double task_time_limit_sec = 60.0;
    int iteration_count = 0;
    int control_loop_delay_count = 0;
    int return_flag = 0;
    bool return_status = true;

    urdf::Model kinova_urdf_model_;
    KDL::Tree kinova_tree_;
    std::ofstream log_file_joint;
    log_file_joint.open("/home/djole/Master/Thesis/GIT/MT_testing/Controller/visualization/archive/single_joint_control.txt");
    assert(log_file_joint.is_open());

    if (!kinova_urdf_model_.initFile("/home/djole/Master/Thesis/GIT/MT_testing/Controller/urdf/kinova-gen3_urdf_V12.urdf"))
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

    // Extract KDL chain from KDL tree
    KDL::Chain robot_chain;
    kinova_tree_.getChain("base_link", "EndEffector_Link", robot_chain);

    // Initialize solvers
    KDL::ChainJntToJacSolver jacob_solver(robot_chain);
    KDL::ChainFkSolverPos_recursive fk_solver_pos(robot_chain);
    KDL::ChainFkSolverVel_recursive fk_solver_vel(robot_chain);
    std::shared_ptr<KDL::ChainIdSolver_RNE> id_solver = std::make_shared<KDL::ChainIdSolver_RNE>(robot_chain, KDL::Vector(0.0, 0.0, -9.81289));
    // ABAG abag(6, true);

    // Setting parameters of the ABAG Controller
    // abag.set_error_alpha((   Eigen::VectorXd(6) << 0.750000, 0.750000, 0.750000, 0.750000, 0.750000, 0.750000).finished());
    // abag.set_bias_threshold((Eigen::VectorXd(6) << 0.000407, 0.000407, 0.000457, 0.000457, 0.000457, 0.000457).finished());
    // abag.set_bias_step((     Eigen::VectorXd(6) << 0.000400, 0.000400, 0.000400, 0.000500, 0.000500, 0.000500).finished());
    // abag.set_gain_threshold((Eigen::VectorXd(6) << 0.502492, 0.502492, 0.502492, 0.502492, 0.502492, 0.502492).finished());
    // abag.set_gain_step((     Eigen::VectorXd(6) << 0.002000, 0.002000, 0.002000, 0.002000, 0.002000, 0.002000).finished());
    // abag.set_min_bias_sat_limit(Eigen::VectorXd::Constant(6, -1.0));
    // abag.set_min_command_sat_limit(Eigen::VectorXd::Constant(6, -1.0));

    // Prepare state and control variables
    KDL::JntArray jnt_command_torque(7), jnt_position(7), jnt_velocity(7), jnt_torque(7), jnt_impedance_torque(7), jnt_current(7), RNE_torque(7);
    KDL::JntArrayVel jnt_position_velocity(7);
    KDL::Jacobian jacobian_end_eff(robot_chain.getNrOfJoints());
    KDL::Frame end_eff_pose;
    KDL::Frame desired_end_eff_pose;
    KDL::FrameVel end_eff_twist;
    KDL::FrameVel desired_end_eff_twist;
    Eigen::VectorXd abag_error_vector(6), abag_command(6);
    abag_error_vector.setZero();
    abag_command.setZero();

    Eigen::Matrix<double, 6, 1> end_eff_force;
    end_eff_force.setZero();

    desired_end_eff_twist.p.v(0) = 0.0;
    desired_end_eff_twist.p.v(1) = 0.0;
    desired_end_eff_twist.p.v(2) = 0.01;

    const KDL::JntArray ZERO_JOINT_ARRAY(7);
    const KDL::Wrenches ZERO_WRENCHES(robot_chain.getNrOfSegments(), KDL::Wrench::Zero());
    const std::vector<double> joint_torque_limits {39.0, 39.0, 39.0, 39.0, 9.0, 9.0, 9.0}; // Nm
    const std::vector<double> cart_force_limit {5.0, 5.0, 5.0, 5.0, 5.0, 5.0}; // N

    k_api::BaseCyclic::Feedback base_feedback;
    k_api::BaseCyclic::Command  base_command;
    auto servoing_mode = k_api::Base::ServoingModeInformation();

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

    try
    {
        // Set the base in low-level servoing mode
        servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
        base->SetServoingMode(servoing_mode);
        base_feedback = base_cyclic->RefreshFeedback();

        // Initialize each actuator to their current position
        for (int i = 0; i < ACTUATOR_COUNT; i++)
            base_command.add_actuators()->set_position(base_feedback.actuators(i).position());

        // Send a first frame
        base_feedback = base_cyclic->Refresh(base_command);
        
        // Set actuators in torque mode
        auto control_mode_message = k_api::ActuatorConfig::ControlModeInformation();
 
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::TORQUE);
        actuator_config->SetControlMode(control_mode_message, 1);

        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::TORQUE);
        actuator_config->SetControlMode(control_mode_message, 2);
    
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::TORQUE);
        actuator_config->SetControlMode(control_mode_message, 3);

        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::TORQUE);
        actuator_config->SetControlMode(control_mode_message, 4);

        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::TORQUE);
        actuator_config->SetControlMode(control_mode_message, 5);

        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::TORQUE);
        actuator_config->SetControlMode(control_mode_message, 6);

        // control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::TORQUE);
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::CURRENT);
        actuator_config->SetControlMode(control_mode_message, 7);

        double torque_cmd = 0.0;
        // double acc = 3.0;
        // torque_cmd = acc * (0.000609 + 0.1389); // acc * (link_inertia + rotor_inertia)
        // // double torque_cmd = acc * 0.000609; // acc * (link_inertia)
        // std::cout << torque_cmd <<std::endl;
        auto control_loop_parameters = k_api::ActuatorConfig::ControlLoopParameters();

        // ####################################################################################################
        // Real-time loop
        // ####################################################################################################
        while (total_time_sec < task_time_limit_sec)
        {
            loop_start_time = std::chrono::steady_clock::now();
            iteration_count++;
            total_time_sec = iteration_count * DT_SEC;

            try
            {
                base_feedback = base_cyclic->RefreshFeedback();
            }
            catch (Kinova::Api::KDetailedException& ex)
            {
                std::cout << "Kortex exception 1: " << ex.what() << std::endl;

                std::cout << "KError error_code 1: " << ex.getErrorInfo().getError().error_code() << std::endl;
                std::cout << "KError sub_code 1: " << ex.getErrorInfo().getError().error_sub_code() << std::endl;
                std::cout << "KError sub_string 1: " << ex.getErrorInfo().getError().error_sub_string() << std::endl;

                // Error codes by themselves are not very verbose if you don't see their corresponding enum value
                // You can use google::protobuf helpers to get the string enum element for every error code and sub-code 
                std::cout << "Error code string equivalent 1: " << Kinova::Api::ErrorCodes_Name(Kinova::Api::ErrorCodes(ex.getErrorInfo().getError().error_code())) << std::endl;
                std::cout << "Error sub-code string equivalent 1: " << Kinova::Api::SubErrorCodes_Name(Kinova::Api::SubErrorCodes(ex.getErrorInfo().getError().error_sub_code())) << std::endl;
                break;
            }
            catch (std::runtime_error& ex2)
            {
                std::cout << "runtime error 1: " << ex2.what() << std::endl;
                break;
            }
            catch(...)
            {
                std::cout << "Unknown error 1." << std::endl;
                break;
            }

            for (int i = 0; i < ACTUATOR_COUNT; i++)
            {
                jnt_position(i) = DEG_TO_RAD(base_feedback.actuators(i).position());
                jnt_velocity(i) = DEG_TO_RAD(base_feedback.actuators(i).velocity());
                jnt_torque(i)   = base_feedback.actuators(i).torque();
                jnt_current(i)  = base_feedback.actuators(i).current_motor();
            }

            // std::cout << jnt_torque(6) << std::endl;

            // Kinova API provides only positive angle values
            // This operation is required to align the logic with our safety monitor
            // We need to convert some angles to negative values
            if (jnt_position(1) > DEG_TO_RAD(180.0)) jnt_position(1) = jnt_position(1) - DEG_TO_RAD(360.0);
            if (jnt_position(3) > DEG_TO_RAD(180.0)) jnt_position(3) = jnt_position(3) - DEG_TO_RAD(360.0);
            if (jnt_position(5) > DEG_TO_RAD(180.0)) jnt_position(5) = jnt_position(5) - DEG_TO_RAD(360.0);

            return_flag = id_solver->CartToJnt(jnt_position, ZERO_JOINT_ARRAY, ZERO_JOINT_ARRAY, ZERO_WRENCHES, RNE_torque);
            if (return_flag != 0) break;

            // return_flag = jacob_solver.JntToJac(jnt_position, jacobian_end_eff);
            // if (return_flag != 0) break;

            // return_flag = fk_solver_pos.JntToCart(jnt_position, end_eff_pose);
            // if (return_flag != 0) break;

            // jnt_position_velocity.q = jnt_position;
            // jnt_position_velocity.qdot = jnt_velocity;
            // return_flag = fk_solver_vel.JntToCart(jnt_position_velocity, end_eff_twist);
            // if (return_flag != 0) break;

            // if (iteration_count == 1)
            // {
            //     desired_end_eff_pose.p(0) = end_eff_pose.p(0) + 0.03;
            //     desired_end_eff_pose.p(1) = end_eff_pose.p(1) + 0.03;
            //     desired_end_eff_pose.p(2) = end_eff_pose.p(2) + 0.03;
            // }

            // abag_error_vector(0) = desired_end_eff_pose.p(0) - end_eff_pose.p(0);
            // abag_error_vector(1) = desired_end_eff_pose.p(1) - end_eff_pose.p(1);
            // abag_error_vector(2) = desired_end_eff_pose.p(2) - end_eff_pose.p(2);

            // abag_error_vector(3) = desired_end_eff_twist.p.v(0) - end_eff_twist.p.v(0);
            // abag_error_vector(4) = desired_end_eff_twist.p.v(1) - end_eff_twist.p.v(1);
            // abag_error_vector(5) = desired_end_eff_twist.p.v(2) - end_eff_twist.p.v(2);
            // std::cout << abag_error_vector.transpose() << std::endl;

            // abag_command = abag.update_state(abag_error_vector).transpose();
            // std::cout << abag_command.transpose() << std::endl;

            // end_eff_force(0) = abag_command(0) * cart_force_limit[0];
            // end_eff_force(1) = abag_command(1) * cart_force_limit[1];
            // end_eff_force(2) = abag_command(2) * cart_force_limit[2];


            // end_eff_force(0) += abag_command(3) * cart_force_limit[3];
            // end_eff_force(1) += abag_command(4) * cart_force_limit[4];
            // end_eff_force(2) += abag_command(5) * cart_force_limit[5];
            // std::cout << end_eff_force.transpose() << std::endl;

            // jnt_impedance_torque.data = jacobian_end_eff.data.transpose() * end_eff_force;
            // std::cout << jnt_impedance_torque.data.transpose() << std::endl;

            int test_joint_index = 6;

            // Kinova support mentioned torque constant values: "The motor Torque gradient (Kt) for the small actuators is 0.076 Nm/A and 0.11Nm/A for the big actuators (+/-10%)"
            double torque_constant = 0.0;
            if (test_joint_index < 4) torque_constant = 5.67;
            else torque_constant = 4.9;

            for (int i = 0; i < ACTUATOR_COUNT; i++)
            {
                if (i != test_joint_index)
                {
                    // jnt_command_torque(i) = 0.0;
                    // jnt_command_torque(i) = jnt_command_torque(i) + jnt_impedance_torque(i);
                    jnt_command_torque(i) = RNE_torque(i);

                    if      (jnt_command_torque(i) >=  joint_torque_limits[i]) jnt_command_torque(i) =  joint_torque_limits[i] - 0.001;
                    else if (jnt_command_torque(i) <= -joint_torque_limits[i]) jnt_command_torque(i) = -joint_torque_limits[i] + 0.001;
                    base_command.mutable_actuators(i)->set_position(base_feedback.actuators(i).position());
                    base_command.mutable_actuators(i)->set_torque_joint(jnt_command_torque(i));
                    // base_command.mutable_actuators(i)->set_current_motor(jnt_command_torque(i) / torque_constant);
                }
            }

            // if (total_time_sec <= 3.5)                               torque_cmd =  0.0;
            // else if (total_time_sec > 3.5 && total_time_sec <= 10.0) torque_cmd =  8.99;
            // else if (total_time_sec > 10.0 && total_time_sec <= 19.0) torque_cmd =  -4.0;
            // else if (total_time_sec > 19.0 && total_time_sec <= 20.0) torque_cmd = 0.0;
            // else break;

            // static bool stable = false;
            // if (std::fabs(jnt_velocity(test_joint_index)) < 0.000001)
            // {
            //     stable = true;
            //     // printf("Stable: %f\n", total_time_sec);
            // }

            torque_cmd += 0.0005; if (torque_cmd > 9.0) torque_cmd = 9.0;
            if (total_time_sec > 8.5) break;
            
            // torque_cmd = 1;
            // torque_cmd = jnt_command_torque(test_joint_index);
            // torque_cmd = RNE_torque(test_joint_index);
            base_command.mutable_actuators(test_joint_index)->set_position(base_feedback.actuators(test_joint_index).position());
            // base_command.mutable_actuators(test_joint_index)->set_torque_joint(torque_cmd);
            base_command.mutable_actuators(test_joint_index)->set_current_motor(torque_cmd / torque_constant);

            // Incrementing identifier ensures actuators can reject out of time frames
            base_command.set_frame_id(base_command.frame_id() + 1);
            if (base_command.frame_id() > 65535) base_command.set_frame_id(0);
            for (int idx = 0; idx < ACTUATOR_COUNT; idx++)
                base_command.mutable_actuators(idx)->set_command_id(base_command.frame_id());

            try
            {
                base_feedback = base_cyclic->Refresh(base_command, 0);
                // RefreshCommand() method is not functioning properly!

                // std::cout << control_loop_parameters.loop_selection() 
                //           << control_loop_parameters.error_saturation() 
                //           << control_loop_parameters.output_saturation() 
                //           << control_loop_parameters.SafetyLimitType 
                //           << control_loop_parameters.SafetyIdentifierBankA
                //           << control_loop_parameters.SafetyStatusValue
                        //   << std::endl;
            }
            catch (k_api::KDetailedException& ex)
            {
                std::cout << "Kortex exception 2: " << ex.what() << std::endl;
                std::cout << "Kortex exception 2: " << ex.what() << std::endl;

                std::cout << "KError error_code 2: " << ex.getErrorInfo().getError().error_code() << std::endl;
                std::cout << "KError sub_code 2: " << ex.getErrorInfo().getError().error_sub_code() << std::endl;
                std::cout << "KError sub_string 2: " << ex.getErrorInfo().getError().error_sub_string() << std::endl;

                // Error codes by themselves are not very verbose if you don't see their corresponding enum value
                // You can use google::protobuf helpers to get the string enum element for every error code and sub-code 
                std::cout << "Error code string equivalent 2: " << Kinova::Api::ErrorCodes_Name(Kinova::Api::ErrorCodes(ex.getErrorInfo().getError().error_code())) << std::endl;
                std::cout << "Error sub-code string equivalent 2: " << Kinova::Api::SubErrorCodes_Name(Kinova::Api::SubErrorCodes(ex.getErrorInfo().getError().error_sub_code())) << std::endl;
                break;
            }
            catch (std::runtime_error& ex2)
            {
                std::cout << "runtime error 2: " << ex2.what() << std::endl;
                break;
            }
            catch(...)
            {
                std::cout << "Unknown error. 2" << std::endl;
                break;
            }

            // static bool detected = false;
            // if ( stable && !detected && std::fabs(jnt_velocity(test_joint_index)) > 0.001)
            // {
            //     detected = true;
            //     std::cout << jnt_velocity(test_joint_index) << " "<< iteration_count << " " << total_time_sec << std::endl;
            //     // std::cout << "Torque command: " << torque_cmd << "      Measured torque: " << jnt_torque(test_joint_index) << "   Measured current: " << jnt_current(test_joint_index)<< std::endl;
            //     std::cout << "Current command: " << torque_cmd / torque_constant << "      Measured torque: " << jnt_torque(test_joint_index) << "   Measured current: " << jnt_current(test_joint_index)<< std::endl;
            // }

            double friction = 1/(2 * std::tanh(5 * jnt_velocity(test_joint_index)));
            // if (friction < -1.3) friction = -1.3;
            // else if (friction > 1.3) friction = 1.3;

            double tau_estimate = -RNE_torque(test_joint_index) - jnt_current(test_joint_index) * torque_constant - friction - 0.5 * (0.011255 + 0.5580);

            log_file_joint << jnt_position(test_joint_index) << " " << jnt_velocity(test_joint_index)                  << " " << jnt_torque(test_joint_index)  << " "
                           << torque_cmd                    << " " << torque_cmd - jnt_torque(test_joint_index)       << " " << jnt_current(test_joint_index) << " "
                           << torque_cmd / torque_constant << " " << jnt_current(test_joint_index) * torque_constant << " " << RNE_torque(test_joint_index)  << " " 
                           << -friction << std::endl;

            // Enforce the constant loop time and count how many times the loop was late
            if (enforce_loop_frequency(DT_MICRO) != 0) control_loop_delay_count++;
        }

        // Set actuators back in position 
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
        for (int actuator_id = 1; actuator_id < ACTUATOR_COUNT + 1; actuator_id++)
            actuator_config->SetControlMode(control_mode_message, actuator_id);
        std::cout << "Torque control example clean exit. Control Loop delay count: \n" << control_loop_delay_count << std::endl;
    }
    catch (k_api::KDetailedException& ex)
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
    servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoing_mode);

    // Wait for a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    log_file_joint.close();
    std::cout << "Total run time: " << total_time_sec << std::endl;

    return return_status;
}

int main(int argc, char **argv)
{
    // Create API objects
    auto error_callback = [](k_api::KError err){ cout << "_________ callback error _________" << err.toString(); };
    
    std::cout << "Creating transport objects" << std::endl;
    auto transport = new k_api::TransportClientTcp();
    auto router = new k_api::RouterClient(transport, error_callback);
    transport->connect(IP_ADDRESS, PORT);

    std::cout << "Creating transport real time objects" << std::endl;
    auto transport_real_time = new k_api::TransportClientUdp();
    auto router_real_time = new k_api::RouterClient(transport_real_time, error_callback);
    transport_real_time->connect(IP_ADDRESS, PORT_REAL_TIME);

    // Set session data connection information
    auto create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username("admin");
    create_session_info.set_password("kinova1_area4251");
    create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
    create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

    // Session manager service wrapper
    std::cout << "Creating sessions for communication" << std::endl;
    auto session_manager = new k_api::SessionManager(router);
    session_manager->CreateSession(create_session_info);
    auto session_manager_real_time = new k_api::SessionManager(router_real_time);
    session_manager_real_time->CreateSession(create_session_info);
    std::cout << "Sessions created" << std::endl;

    // Create services
    auto base = new k_api::Base::BaseClient(router);
    auto base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router_real_time);
    auto actuator_config = new k_api::ActuatorConfig::ActuatorConfigClient(router);

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