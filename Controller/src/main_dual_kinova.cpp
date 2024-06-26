/*
Author(s): Djordje Vukcevic, Sven Schneider
Institute: Hochschule Bonn-Rhein-Sieg
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
#include <kinova_mediator.hpp>
#include <state_specification.hpp>
#include <dynamics_controller.hpp>
#include <solver_recursive_newton_euler.hpp>
#include <fk_vereshchagin.hpp>
#include <geometry_utils.hpp>
#include <model_prediction.hpp>
#include <safety_monitor.hpp>
#include <finite_state_machine.hpp>
#include <motion_profile.hpp>

#define IP_ADDRESS_1 "192.168.1.10"
#define IP_ADDRESS_2 "192.168.1.12"
#define PORT 10000

enum desired_pose
{
    CANDLE         = 0,
    HOME           = 1,
    RETRACT        = 2,
    PACKAGING      = 3,
    HOME_FORWARD   = 4,
    HOME_BACK      = 5,
    APPROACH_TABLE = 6
};

enum path_types
{
    SINE_PATH = 0,
    STEP_PATH = 1,
    INF_SIGN_PATH = 2
};

// Waiting time during actions
constexpr auto TIMEOUT_DURATION      = std::chrono::seconds{20};
std::chrono::steady_clock::time_point loop_start_time;
std::chrono::duration <double, std::micro> loop_interval{};

const int SECOND                     = 1000000; // Number of microseconds in one second
const int MILLISECOND                = 1000; // Number of microseconds in one millisecond
const int JOINTS                     = 7;
const int NUMBER_OF_CONSTRAINTS      = 6;
const int desired_dynamics_interface = dynamics_interface::CART_ACCELERATION;
int RATE_HZ                          = 1000; // Hz
int motion_profile_id                = m_profile::CONSTANT;
int path_type                        = path_types::STEP_PATH;
int desired_pose_id                  = desired_pose::HOME;
int desired_task_model               = task_model::full_pose;
int desired_control_mode             = control_mode::TORQUE;
int environment                      = kinova_environment::SIMULATION;
int robot_model_id                   = kinova_model::URDF;
double time_horizon_amplitude        = 2.5;
double tube_speed                    = 0.01;
double tube_force                    = 0.03;
double desired_null_space_angle      = 90.0; // Unit degrees
double task_time_limit_sec           = 600.0;
double contact_threshold_linear      = 35.0; // N
double contact_threshold_angular     = 2.0; // Nm
bool log_data_1                      = false;
bool log_data_2                      = false;
bool use_estimated_external_wrench   = false;
bool control_null_space              = false;
bool control_null_space_moveConstrained = false;
bool compensate_gravity              = false;
bool use_mass_alternation            = false;

std::vector<bool> control_dims       = {true, true, true, // Linear
                                        false, false, false}; // Angular
std::vector<bool> control_dims_moveConstrained = {true, true, true, // Linear
                                                  true, true, false}; // Angular
// Last parameter: Numer of points
std::vector<double> path_parameters = {0.5, 0.5, 0.05, 0.008, 40}; // last parameter must be min 2 (number of points)
std::vector<double> path_parameters_moveConstrained = {0.5, 0.5, 0.05, 0.008, 40};
std::vector<double> tube_start_position   = {0.0, 0.0, 0.0};
std::vector<double> tube_tolerances       = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

// Tube tolerances: x pos,    y pos,      z force, 
//                  x torque, y torque,   null-space, 
//                  x vel,    z_a pos/vel
std::vector<double> tube_tolerances_moveConstrained = {0.003, 0.03, 0.003,
                                                       0.005, 0.005, 25.0,
                                                       0.003, 0.001};

std::vector< std::vector<double> > tube_path_points(path_parameters[4], std::vector<double>(3, 0.0));
std::vector< std::vector<double> > path_poses(path_parameters[4] - 1,   std::vector<double>(12, 0.0));
std::vector< std::vector<double> > tube_path_points_moveConstrained(path_parameters_moveConstrained[4], std::vector<double>(3, 0.0));
std::vector< std::vector<double> > path_poses_moveConstrained(path_parameters_moveConstrained[4] - 1, std::vector<double>(12, 0.0));

Eigen::VectorXd max_command                 = (Eigen::VectorXd(NUMBER_OF_CONSTRAINTS) << 20.0, 20.0, 20.0, 20.0, 20.0, 20.0).finished();
Eigen::VectorXd max_command_moveConstrained = (Eigen::VectorXd(NUMBER_OF_CONSTRAINTS) << 20.0, 20.0, 1.0, 1.0, 1.0, 10.0).finished();

// Full Pose ABAG parameters
const Eigen::VectorXd error_alpha         = (Eigen::VectorXd(NUMBER_OF_CONSTRAINTS) \
                                            << 0.900000, 0.900000, 0.900000, 
                                               0.900000, 0.900000, 0.900000).finished();
const Eigen::VectorXd bias_threshold      = (Eigen::VectorXd(NUMBER_OF_CONSTRAINTS) \
                                            << 0.000407, 0.000407, 0.000407, 
                                               0.000500, 0.000500, 0.000500).finished();
const Eigen::VectorXd bias_step           = (Eigen::VectorXd(NUMBER_OF_CONSTRAINTS) \
                                            << 0.000495, 0.000495, 0.000495, 
                                               0.000800, 0.000800, 0.000800).finished();
const Eigen::VectorXd gain_threshold      = (Eigen::VectorXd(NUMBER_OF_CONSTRAINTS) \
                                            << 0.552492, 0.552492, 0.552492, 
                                               0.650000, 0.650000, 0.650000).finished();
const Eigen::VectorXd gain_step           = (Eigen::VectorXd(NUMBER_OF_CONSTRAINTS) \
                                            << 0.003152, 0.003152, 0.003152, 
                                               0.002500, 0.002500, 0.002500).finished();

// moveGuarded-torque ABAG parameters
const Eigen::VectorXd error_alpha_1         = (Eigen::VectorXd(NUMBER_OF_CONSTRAINTS) \
                                            << 0.900000, 0.900000, 0.900000, 
                                               0.900000, 0.900000, 0.900000).finished();
const Eigen::VectorXd bias_threshold_1      = (Eigen::VectorXd(NUMBER_OF_CONSTRAINTS) \
                                            << 0.000457, 0.000407, 0.000407, 
                                               0.000500, 0.000500, 0.000500).finished();
const Eigen::VectorXd bias_step_1           = (Eigen::VectorXd(NUMBER_OF_CONSTRAINTS) \
                                            << 0.000500, 0.000400, 0.000400, 
                                               0.000800, 0.000800, 0.000800).finished();
const Eigen::VectorXd gain_threshold_1      = (Eigen::VectorXd(NUMBER_OF_CONSTRAINTS) \
                                            << 0.502492, 0.502492, 0.502492, 
                                               0.650000, 0.650000, 0.650000).finished();
const Eigen::VectorXd gain_step_1           = (Eigen::VectorXd(NUMBER_OF_CONSTRAINTS) \
                                            << 0.002552, 0.002552, 0.002552, 
                                               0.002500, 0.002500, 0.002500).finished();

// moveTo-torque ABAG parameters
const Eigen::VectorXd error_alpha_2         = (Eigen::VectorXd(NUMBER_OF_CONSTRAINTS) \
                                            << 0.900000, 0.900000, 0.900000, 
                                               0.900000, 0.900000, 0.900000).finished();
const Eigen::VectorXd bias_threshold_2      = (Eigen::VectorXd(NUMBER_OF_CONSTRAINTS) \
                                            << 0.000457, 0.000407, 0.000407, 
                                               0.000407, 0.000407, 0.000407).finished();
const Eigen::VectorXd bias_step_2           = (Eigen::VectorXd(NUMBER_OF_CONSTRAINTS) \
                                            << 0.000500, 0.000400, 0.000400, 
                                               0.000400, 0.000400, 0.000400).finished();
const Eigen::VectorXd gain_threshold_2      = (Eigen::VectorXd(NUMBER_OF_CONSTRAINTS) \
                                            << 0.502492, 0.502492, 0.502492, 
                                               0.502492, 0.502492, 0.502492).finished();
const Eigen::VectorXd gain_step_2           = (Eigen::VectorXd(NUMBER_OF_CONSTRAINTS) \
                                            << 0.002552, 0.002552, 0.002552, 
                                               0.002552, 0.002552, 0.002552).finished();

// moveTo-follow_path-torque ABAG parameters
const Eigen::VectorXd error_alpha_3         = (Eigen::VectorXd(NUMBER_OF_CONSTRAINTS) \
                                            << 0.850000, 0.850000, 0.850000, 
                                               0.850000, 0.850000, 0.850000).finished();
const Eigen::VectorXd bias_threshold_3      = (Eigen::VectorXd(NUMBER_OF_CONSTRAINTS) \
                                            << 0.000457, 0.000407, 0.000407, 
                                               0.001007, 0.001007, 0.001007).finished();
const Eigen::VectorXd bias_step_3           = (Eigen::VectorXd(NUMBER_OF_CONSTRAINTS) \
                                            << 0.000550, 0.000550, 0.000550, 
                                               0.003495, 0.003495, 0.003495).finished();
const Eigen::VectorXd gain_threshold_3      = (Eigen::VectorXd(NUMBER_OF_CONSTRAINTS) \
                                            << 0.502492, 0.454092, 0.454092, 
                                               0.252492, 0.252492, 0.252492).finished();
const Eigen::VectorXd gain_step_3           = (Eigen::VectorXd(NUMBER_OF_CONSTRAINTS) \
                                            << 0.003152, 0.003552, 0.003552, 
                                               0.015152, 0.015152, 0.015152).finished();

// moveTo_weight_compensation-torque ABAG parameters
const Eigen::VectorXd error_alpha_4         = (Eigen::VectorXd(NUMBER_OF_CONSTRAINTS) \
                                            << 0.900000, 0.900000, 0.900000, 
                                               0.850000, 0.850000, 0.850000).finished();
const Eigen::VectorXd bias_threshold_4      = (Eigen::VectorXd(NUMBER_OF_CONSTRAINTS) \
                                            << 0.000550, 0.000457, 0.000457, 
                                               0.001007, 0.001007, 0.001007).finished();
const Eigen::VectorXd bias_step_4           = (Eigen::VectorXd(NUMBER_OF_CONSTRAINTS) \
                                            << 0.000500, 0.000450, 0.000450, 
                                               0.003495, 0.003495, 0.003495).finished();
const Eigen::VectorXd gain_threshold_4      = (Eigen::VectorXd(NUMBER_OF_CONSTRAINTS) \
                                            << 0.452492, 0.452492, 0.452492, 
                                               0.252492, 0.252492, 0.252492).finished();
const Eigen::VectorXd gain_step_4           = (Eigen::VectorXd(NUMBER_OF_CONSTRAINTS) \
                                            << 0.002600, 0.002600, 0.002600, 
                                               0.015152, 0.015152, 0.015152).finished();

// moveConstrained-follow_path-torque ABAG parameters
const Eigen::VectorXd error_alpha_5         = (Eigen::VectorXd(NUMBER_OF_CONSTRAINTS) \
                                            << 0.900000, 0.900000, 0.900000,
                                               0.900000, 0.900000, 0.900000).finished();
const Eigen::VectorXd bias_threshold_5      = (Eigen::VectorXd(NUMBER_OF_CONSTRAINTS) \
                                            << 0.000457, 0.000407, 0.000407, 
                                               0.000507, 0.000507, 0.000457).finished();
const Eigen::VectorXd bias_step_5           = (Eigen::VectorXd(NUMBER_OF_CONSTRAINTS) \
                                            << 0.000500, 0.000400, 0.000905, 
                                               0.000495, 0.000495, 0.000500).finished();
const Eigen::VectorXd gain_threshold_5      = (Eigen::VectorXd(NUMBER_OF_CONSTRAINTS) \
                                            << 0.502492, 0.502492, 0.452492, 
                                               0.452492, 0.452492, 0.502492).finished();
const Eigen::VectorXd gain_step_5           = (Eigen::VectorXd(NUMBER_OF_CONSTRAINTS) \
                                            << 0.002552, 0.002552, 0.003652, 
                                               0.002052, 0.002052, 0.002552).finished();

// Stop Motion control parameters used by the ABAG -> parameters specific each robot type... test these values
const Eigen::VectorXd STOP_MOTION_ERROR_ALPHA    = (Eigen::VectorXd(JOINTS) << 0.800000, 0.800000, 0.800000, 0.800000, 0.800000, 0.800000, 0.800000).finished();
const Eigen::VectorXd STOP_MOTION_BIAS_THRESHOLD = (Eigen::VectorXd(JOINTS) << 0.000557, 0.006000, 0.000557, 0.006500, 0.000457, 0.006500, 0.000457).finished();
const Eigen::VectorXd STOP_MOTION_BIAS_STEP      = (Eigen::VectorXd(JOINTS) << 0.000900, 0.002500, 0.000900, 0.002000, 0.000500, 0.002000, 0.000500).finished();
const Eigen::VectorXd STOP_MOTION_GAIN_THRESHOLD = (Eigen::VectorXd(JOINTS) << 0.602492, 0.500000, 0.602492, 0.500000, 0.602492, 0.500000, 0.602492).finished();
const Eigen::VectorXd STOP_MOTION_GAIN_STEP      = (Eigen::VectorXd(JOINTS) << 0.005552, 0.010552, 0.005552, 0.010552, 0.003552, 0.010552, 0.003552).finished();

const Eigen::VectorXd min_bias_sat               = Eigen::VectorXd::Constant(6, -1.0);
const Eigen::VectorXd min_command_sat            = Eigen::VectorXd::Constant(6, -1.0);
const Eigen::VectorXd null_space_abag_parameters = (Eigen::VectorXd(6) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1).finished(); // Last param is max command
const Eigen::VectorXd null_space_abag_parameters_moveConstrained = (Eigen::VectorXd(6) << 0.850000, 0.000507, 0.000455, 0.452492, 0.001552, 250.0).finished(); // Last param is max command

//  Parameters for weight compensation: x_bias-offset (-1.0 <-> 1.0), y_bias-offset, z_bias-offset, K proportional, error-tube(0.0 <-> 1.0),
//                                      bias-variance, gain-variance, bias slope, 
//                                      control-period, x_max_trigger_count, y_max_trigger_count, z_max_trigger_count
const Eigen::VectorXd compensation_parameters = (Eigen::VectorXd(12) << -0.08, -0.07, 0.0, 1.2, 0.015,
                                                                         0.00016, 0.0025, 0.00002,
                                                                         60, 6, 3, 3).finished();


const Eigen::VectorXd wrench_estimation_gain = (Eigen::VectorXd(7) << 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0).finished();

// Create an event listener that will set the promise action event to the exit value
// Will set promise to either END or ABORT
// Use finish_promise.get_future.get() to wait and get the value
std::function<void(Kinova::Api::Base::ActionNotification)> create_event_listener_by_promise(std::promise<Kinova::Api::Base::ActionEvent>& finish_promise)
{
    return [&finish_promise] (Kinova::Api::Base::ActionNotification notification)
    {
        const auto action_event = notification.action_event();
        switch(action_event)
        {
            case Kinova::Api::Base::ActionEvent::ACTION_END:
            case Kinova::Api::Base::ActionEvent::ACTION_ABORT:
                finish_promise.set_value(action_event);
                break;
            default:
                break;
        }
    };
}

// Define the callback function used in Refresh_callback
auto lambda_fct_callback = [](const Kinova::Api::Error &err, const Kinova::Api::BaseCyclic::Feedback data)
{
    // We are printing the data of the moving actuator just for the example purpose,
    // avoid this in a real-time loop
    std::string serialized_data;
    google::protobuf::util::MessageToJsonString(data.actuators(6), &serialized_data);
    std::cout << serialized_data << std::endl << std::endl;
};

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

void rotate_joint(kinova_mediator &robot_driver, const int joint, const double rate)
{
    robot_driver.set_control_mode(control_mode::VELOCITY);
    KDL::JntArray rotate_joint(JOINTS);
    rotate_joint(joint) = rate; 
    robot_driver.set_joint_velocities(rotate_joint);
    if (environment != kinova_environment::SIMULATION) usleep(5000 * MILLISECOND);
}

int go_to(kinova_mediator &robot_driver_1, kinova_mediator &robot_driver_2, const int desired_pose_)
{
    std::vector<double> configuration_array(7, 0.0);
    switch (desired_pose_) // Angle value are in units of degree
    {
        case desired_pose::CANDLE:
            configuration_array = std::vector<double> {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            break;
        case desired_pose::APPROACH_TABLE:
            configuration_array = std::vector<double> {0.001, 42.017, 179.56, 220.641, 2.761, 1.965, 88.101};
            break;
        case desired_pose::HOME_BACK:
            configuration_array = std::vector<double> {356.129, 304.126, 181.482, 250.087, 2.852, 328.367, 87.817};
            break;
        case desired_pose::PACKAGING:
            configuration_array = std::vector<double> {0.0, 330.0, 180.0, 214.0, 0.0, 115.0, 270.0};
            break;
        case desired_pose::RETRACT:
            configuration_array = std::vector<double> {0.0, 340.0, 180.0, 214.0, 0.0, 310.0, 90.0};
            break;       
        case desired_pose::HOME:
            configuration_array = std::vector<double> {0.0, 15.0, 180.0, 230.0, 0.0, 55.0, 90.0};
            break;
        default: return -1;
    }
    if (environment != kinova_environment::SIMULATION)
    {
        // Extract user password from a file
        fstream kinova_file;
        string kinova_passwd_1, kinova_passwd_2;
        kinova_file.open("/home/djole/Master/Thesis/GIT/MT_testing/kinova_passwd.txt", ios::in);
        if (kinova_file.is_open())
        {
            getline(kinova_file, kinova_passwd_1);
            while (getline(kinova_file, kinova_passwd_2)) {}
            kinova_file.close();
        }

        auto error_callback = [](Kinova::Api::KError err){ cout << "_________ callback error _________" << err.toString(); };
        
        // Kinova 1 Create API objects
        auto transport_1 = new Kinova::Api::TransportClientTcp();
        auto router_1 = new Kinova::Api::RouterClient(transport_1, error_callback);
        transport_1->connect(IP_ADDRESS_1, PORT);
        // Set session data connection information
        auto create_session_info_1 = Kinova::Api::Session::CreateSessionInfo();
        create_session_info_1.set_username("admin");
        create_session_info_1.set_password(kinova_passwd_1);
        create_session_info_1.set_session_inactivity_timeout(200);   // (milliseconds)
        create_session_info_1.set_connection_inactivity_timeout(200); // (milliseconds)
        // Session manager service wrapper
        auto session_manager_1 = new Kinova::Api::SessionManager(router_1);
        session_manager_1->CreateSession(create_session_info_1);
        // Create services
        auto base_1 = new Kinova::Api::Base::BaseClient(router_1);


        // Kinova 2 Create API objects
        auto transport_2 = new Kinova::Api::TransportClientTcp();
        auto router_2 = new Kinova::Api::RouterClient(transport_2, error_callback);
        transport_2->connect(IP_ADDRESS_2, PORT);

        // Set session data connection information
        auto create_session_info_2 = Kinova::Api::Session::CreateSessionInfo();
        create_session_info_2.set_username("admin");
        create_session_info_2.set_password(kinova_passwd_2);
        create_session_info_2.set_session_inactivity_timeout(200);   // (milliseconds)
        create_session_info_2.set_connection_inactivity_timeout(200); // (milliseconds)
        // Session manager service wrapper
        auto session_manager_2 = new Kinova::Api::SessionManager(router_2);
        session_manager_2->CreateSession(create_session_info_2);
        // Create services
        auto base_2 = new Kinova::Api::Base::BaseClient(router_2);

        // Make sure the arm is in Single Level Servoing before executing an Action
        auto servoingMode = Kinova::Api::Base::ServoingModeInformation();
        servoingMode.set_servoing_mode(Kinova::Api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
        base_1->SetServoingMode(servoingMode);
        base_2->SetServoingMode(servoingMode);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        auto constrained_joint_angles = Kinova::Api::Base::ConstrainedJointAngles();
        auto joint_angles = constrained_joint_angles.mutable_joint_angles();
        auto actuator_count = base_1->GetActuatorCount();

        for (size_t i = 0; i < actuator_count.count(); ++i) 
        {
            auto joint_angle = joint_angles->add_joint_angles();
            joint_angle->set_joint_identifier(i);
            joint_angle->set_value(configuration_array[i]);
        }

        // Connect to notification action topic (Promise alternative)
        // See cartesian examples for Reference alternative
        std::promise<Kinova::Api::Base::ActionEvent> finish_promise_1;
        auto finish_future_1 = finish_promise_1.get_future();
        auto promise_notification_handle_1 = base_1->OnNotificationActionTopic(
            create_event_listener_by_promise(finish_promise_1),
            Kinova::Api::Common::NotificationOptions()
        );
        std::promise<Kinova::Api::Base::ActionEvent> finish_promise_2;
        auto finish_future_2 = finish_promise_2.get_future();
        auto promise_notification_handle_2 = base_2->OnNotificationActionTopic(
            create_event_listener_by_promise(finish_promise_2),
            Kinova::Api::Common::NotificationOptions()
        );

        // std::cout << "Reaching joint angles..." << std::endl;
        base_1->PlayJointTrajectory(constrained_joint_angles);
        base_2->PlayJointTrajectory(constrained_joint_angles);

        // Wait for future value from promise (Promise alternative)
        // See cartesian examples for Reference alternative
        const auto status_1 = finish_future_1.wait_for(TIMEOUT_DURATION);
        const auto status_2 = finish_future_2.wait_for(TIMEOUT_DURATION);
        base_1->Unsubscribe(promise_notification_handle_1);
        base_2->Unsubscribe(promise_notification_handle_2);

        if (status_1 != std::future_status::ready)
        {
            std::cout << "Timeout on action notification wait" << std::endl;
            std::cout << "Can't reach safe position, exiting" << std::endl;

            // Close API session
            session_manager_1->CloseSession();

            // Deactivate the router and cleanly disconnect from the transport object
            router_1->SetActivationStatus(false);
            transport_1->disconnect();

            // Destroy the API
            delete base_1;
            delete session_manager_1;
            delete router_1;
            delete transport_1;
            return -1;
        }

        if (status_2 != std::future_status::ready)
        {
            std::cout << "Timeout on action notification wait" << std::endl;
            std::cout << "Can't reach safe position, exiting" << std::endl;

            // Close API session
            session_manager_2->CloseSession();

            // Deactivate the router and cleanly disconnect from the transport object
              router_2->SetActivationStatus(false);
            transport_2->disconnect();

            // Destroy the API
            delete base_2;
            delete session_manager_2;
            delete router_2;
            delete transport_2;
            return -1;
        }

        // const auto promise_event_1 = finish_future_1.get();
        // const auto promise_event_2 = finish_future_2.get();

        // std::cout << "Joint angles reached" << std::endl;
        // std::cout << "Promise value : " << Kinova::Api::Base::ActionEvent_Name(promise_event) << std::endl;


        // Close API session
        session_manager_1->CloseSession();
        session_manager_2->CloseSession();

        // Deactivate the router and cleanly disconnect from the transport object
        router_1->SetActivationStatus(false);
        router_2->SetActivationStatus(false);

        transport_1->disconnect();
        transport_2->disconnect();

        // Destroy the API
        delete base_1;
        delete base_2;
        delete session_manager_1;
        delete session_manager_2;
        delete router_1;
        delete router_2;
        delete transport_1;
        delete transport_2;

        // printf("High-Level Control Completed\n");
    }
    else
    {
        robot_driver_1.initialize(robot_model_id, environment, robot_id::KINOVA_GEN3_1, 1.0 / static_cast<double>(RATE_HZ));
        if (!robot_driver_1.is_initialized())
        {
            printf("Robot 1 is not initialized\n");
            return -1;
        }

        robot_driver_2.initialize(robot_model_id, environment, robot_id::KINOVA_GEN3_2, 1.0 / static_cast<double>(RATE_HZ));
        if (!robot_driver_2.is_initialized())
        {
            printf("Robot 2 is not initialized\n");
            return -1;
        }

        if (robot_driver_1.set_control_mode(control_mode::POSITION) == -1)
        {
            printf("Incorrect control mode 1\n");
            return -1;
        }

        if (robot_driver_2.set_control_mode(control_mode::POSITION) == -1)
        {
            printf("Incorrect control mode 2\n");
            return -1;
        }

        KDL::JntArray config(JOINTS);
        for (int i = 0; i < JOINTS; i++) 
            config(i) = DEG_TO_RAD(configuration_array[i]);

        // Kinova API provides only positive angle values
        // This operation is required to align the logic with our safety monitor
        // We need to convert some angles to negative values
        if (config(1) > DEG_TO_RAD(180.0)) config(1) -= DEG_TO_RAD(360.0);
        if (config(3) > DEG_TO_RAD(180.0)) config(3) -= DEG_TO_RAD(360.0);
        if (config(5) > DEG_TO_RAD(180.0)) config(5) -= DEG_TO_RAD(360.0);

        robot_driver_1.set_joint_positions(config);
        robot_driver_2.set_joint_positions(config);
    }
    return 0;
}

int define_task(dynamics_controller *dyn_controller)
{
    std::vector<double> desired_ee_pose(12, 0.0);
    switch (desired_pose_id)
    {
        case desired_pose::CANDLE:
            tube_start_position = std::vector<double>{0.0, -0.0248591, 0.92586};
            desired_ee_pose     = { 0.0, -0.0248591, 1.12586, // Linear: Vector
                                    1.0, 0.0, 0.0, // Angular: Rotation matrix
                                    0.0, 1.0, 0.0,
                                    0.0, 0.0, 1.0};
            break;

        case desired_pose::APPROACH_TABLE:
            tube_start_position = std::vector<double>{0.275073, 0.00364068, 0.177293};
            desired_ee_pose     = { 0.275073,  0.00364068,    0.177293, // Linear: Vector
                                    -0.0207239,   -0.999733,   -0.010216, // Angular: Rotation matrix
                                    0.999763,  -0.0206545, -0.00685178,
                                    0.00663895,  -0.0103556,    0.999924};
            break;

        case desired_pose::HOME_FORWARD:
            tube_start_position = std::vector<double>{0.39514, 0.00134662, 0.433724};
            desired_ee_pose     = { 0.39514, 0.00134662, 0.433724, // Linear: Vector
                                    0.0, 0.0, -1.0, // Angular: Rotation matrix
                                    1.0, 0.0, 0.0,
                                    0.0, -1.0, 0.0};
            break;

        case desired_pose::HOME_BACK:
            tube_start_position = std::vector<double>{0.0125206, -0.00995057, 0.713622};
            desired_ee_pose     = { 0.0125206, -0.00995057, 0.713622, // Linear: Vector
                                    -0.0266768,   0.0747393,   -0.996846, // Angular: Rotation matrix
                                     0.999461,   0.0210588,  -0.0251679,
                                     0.0191113,   -0.996981,  -0.0752609};
            break;

        case desired_pose::RETRACT:
            tube_start_position = std::vector<double>{0.117804, 0.00134572, 0.489747};
            desired_ee_pose     = { 0.117804, 0.00134572, 0.389747, // Linear: Vector
                                    0.0, -0.997564, -0.0697565, // Angular: Rotation matrix
                                    1.0,  0.0,       0.0,
                                    0.0, -0.0697565, 0.997564};
            break;

        case desired_pose::HOME:
            tube_start_position = std::vector<double>{ 0.395153, 0.00136493, 0.433647};
            desired_ee_pose     = { 0.49514, 0.00136493, 0.433647, // Linear: Vector
                                    0.0, 0.0, -1.0, // Angular: Rotation matrix
                                    1.0, 0.0, 0.0,
                                    0.0, -1.0, 0.0};
            break;

        default: return -1;
    }

    switch (desired_task_model)
    {
        case task_model::moveConstrained_follow_path:
            switch (path_type)
            {
                case path_types::STEP_PATH:
                    motion_profile::draw_step_xy(tube_path_points_moveConstrained, 6, 0.07, desired_ee_pose[0], desired_ee_pose[1], desired_ee_pose[2]);
                    break;

                case path_types::INF_SIGN_PATH:
                    motion_profile::draw_inf_sign_xy(tube_path_points_moveConstrained, 0.15, 0.2, 0.7, 1.0, desired_ee_pose[0], desired_ee_pose[1], desired_ee_pose[2]);
                    break;

                case path_types::SINE_PATH:
                    motion_profile::draw_sine_xy(tube_path_points_moveConstrained, path_parameters_moveConstrained[0], path_parameters_moveConstrained[1],
                                                 path_parameters_moveConstrained[2], path_parameters_moveConstrained[3], 
                                                 desired_ee_pose[0], desired_ee_pose[1], desired_ee_pose[2]);
                    break;

                default:
                    printf("Unsupported path type");
                    return false;
            }

            dyn_controller->define_moveConstrained_follow_path_task(std::vector<bool>{control_dims_moveConstrained[0], control_dims_moveConstrained[1], control_dims_moveConstrained[2], // Linear
                                                                                      control_dims_moveConstrained[3], control_dims_moveConstrained[4], control_dims_moveConstrained[5]},// Angular
                                                                    tube_path_points_moveConstrained,
                                                                    tube_tolerances_moveConstrained,
                                                                    tube_speed,
                                                                    tube_force,
                                                                    90.5, 90.4, //contact_threshold linear and angular
                                                                    task_time_limit_sec,// time_limit
                                                                    control_null_space_moveConstrained,
                                                                    desired_null_space_angle,
                                                                    path_poses_moveConstrained); // TF pose
            break;

        case task_model::moveTo_follow_path:
            switch (path_type)
            {
                case path_types::STEP_PATH:
                    motion_profile::draw_step_xy(tube_path_points, 6, 0.001, desired_ee_pose[0], desired_ee_pose[1], desired_ee_pose[2]);
                    break;
                
                case path_types::INF_SIGN_PATH:
                    motion_profile::draw_inf_sign_xy(tube_path_points, 0.5, 0.4, 0.18, 0.5, desired_ee_pose[0], desired_ee_pose[1], desired_ee_pose[2]);
                    break;

                case path_types::SINE_PATH:
                    motion_profile::draw_sine_xy(tube_path_points, path_parameters[0], path_parameters[1],
                                                 path_parameters[2], path_parameters[3], 
                                                 desired_ee_pose[0], desired_ee_pose[1], desired_ee_pose[2]);
                    break;
                
                default:
                    printf("Unsupported path type");
                    return -1;
                    break;
            }

            dyn_controller->define_moveTo_follow_path_task(std::vector<bool>{control_dims[0], control_dims[1], control_dims[2], // Linear
                                                                             control_dims[3], control_dims[4], control_dims[5]},// Angular
                                                            tube_path_points,
                                                            tube_tolerances,
                                                            tube_speed,
                                                            contact_threshold_linear, contact_threshold_angular,
                                                            task_time_limit_sec,// time_limit
                                                            control_null_space,
                                                            desired_null_space_angle,
                                                            path_poses); // TF pose
            break;

        case task_model::moveTo:
            dyn_controller->define_moveTo_task(std::vector<bool>{control_dims[0], control_dims[1], control_dims[2], // Linear
                                                                 control_dims[3], control_dims[4], control_dims[5]},// Angular
                                               tube_start_position,
                                               tube_tolerances,
                                               tube_speed,
                                               contact_threshold_linear, contact_threshold_angular,
                                               task_time_limit_sec,// time_limit
                                               control_null_space,
                                               desired_null_space_angle,
                                               desired_ee_pose); // TF pose
            break;

        case task_model::moveGuarded:
            dyn_controller->define_moveGuarded_task(std::vector<bool>{control_dims[0], control_dims[1], control_dims[2], // Linear
                                                                      control_dims[3], control_dims[4], control_dims[5]},// Angular
                                                    tube_start_position,
                                                    tube_tolerances,
                                                    tube_speed,
                                                    contact_threshold_linear, contact_threshold_angular,
                                                    task_time_limit_sec,// time_limit
                                                    control_null_space,
                                                    desired_null_space_angle,
                                                    desired_ee_pose); // TF pose
            break;

        case task_model::moveTo_weight_compensation:
            dyn_controller->define_moveTo_weight_compensation_task(std::vector<bool>{control_dims[0], control_dims[1], control_dims[2], // Linear
                                                                                     control_dims[3], control_dims[4], control_dims[5]},// Angular
                                                                   tube_start_position,
                                                                   tube_tolerances,
                                                                   tube_speed,
                                                                   contact_threshold_linear, contact_threshold_angular,
                                                                   task_time_limit_sec,// time_limit
                                                                   control_null_space,
                                                                   desired_null_space_angle,
                                                                   use_mass_alternation,
                                                                   desired_ee_pose); // TF pose
            break;

        case task_model::full_pose:
            dyn_controller->define_full_pose_task(std::vector<bool>{control_dims[0], control_dims[1], control_dims[2], // Linear
                                                                    control_dims[3], control_dims[4], control_dims[5]}, // Angular
                                                  desired_ee_pose,
                                                  contact_threshold_linear, contact_threshold_angular,
                                                  task_time_limit_sec,
                                                  control_null_space,
                                                  desired_null_space_angle,
                                                  tube_tolerances[7]); // Null space tolerance
            break;

        case task_model::gravity_compensation:
            dyn_controller->define_gravity_compensation_task(task_time_limit_sec);
            break;

        default:
            assert(("Unsupported task model", false));
            return -1;
            break;
    }

    return 0;
}

void run_test(kinova_mediator &robot_driver_1, kinova_mediator &robot_driver_2)
{
    const int DT_MICRO = SECOND / RATE_HZ;
    const double DT_SEC = 1.0 / static_cast<double>(RATE_HZ);
    int id_solver_result = 0;
    int return_flag = 0;
    int iteration_count = 0;
    double total_time_sec = 0.0;
    int control_loop_delay_count = 0;
    
    KDL::JntArray torque_command_1(7), joint_pos_1(7), joint_vel_1(7), joint_torque_1(7),
                  torque_command_2(7), joint_pos_2(7), joint_vel_2(7), joint_torque_2(7);
    const KDL::JntArray ZERO_JOINT_ARRAY(7);

    KDL::Chain robot_chain_1 = robot_driver_1.get_robot_model();
    KDL::Chain robot_chain_2 = robot_driver_2.get_robot_model();

    // Above main chain is prepared for vereshchagin (nj == ns) but full contains additional segments
    KDL::Chain robot_chain_full_1 = robot_driver_1.get_full_robot_model();
    KDL::Chain robot_chain_full_2 = robot_driver_2.get_full_robot_model();

    const KDL::Wrenches zero_wrenches_full_model_1(robot_chain_full_1.getNrOfSegments(), KDL::Wrench::Zero());
    const KDL::Wrenches zero_wrenches_full_model_2(robot_chain_full_2.getNrOfSegments(), KDL::Wrench::Zero());

    std::shared_ptr<KDL::Solver_RNE> id_solver_1 = std::make_shared<KDL::Solver_RNE>(robot_chain_full_1, -1 * robot_driver_1.get_root_acceleration().vel, robot_driver_1.get_joint_inertia(), robot_driver_1.get_joint_torque_limits(), true);
    std::shared_ptr<KDL::Solver_RNE> id_solver_2 = std::make_shared<KDL::Solver_RNE>(robot_chain_full_2, -1 * robot_driver_2.get_root_acceleration().vel, robot_driver_2.get_joint_inertia(), robot_driver_2.get_joint_torque_limits(), true);

    // Real-time loop
    while (total_time_sec < task_time_limit_sec)
    {
        loop_start_time = std::chrono::steady_clock::now();
        iteration_count++;
        total_time_sec = iteration_count * DT_SEC;

        robot_driver_1.get_joint_state(joint_pos_1, joint_vel_1, joint_torque_1);
        robot_driver_2.get_joint_state(joint_pos_2, joint_vel_2, joint_torque_2);

        // Compute dynamics
        id_solver_result = id_solver_1->CartToJnt(joint_pos_1, ZERO_JOINT_ARRAY, ZERO_JOINT_ARRAY, zero_wrenches_full_model_1, torque_command_1);
        if (id_solver_result != 0)
        {
            robot_driver_1.stop_robot_motion();
            robot_driver_2.stop_robot_motion();
            printf("Robot stoped: error in dynamics 1\n");
            return;
        }

        id_solver_result = id_solver_2->CartToJnt(joint_pos_2, ZERO_JOINT_ARRAY, ZERO_JOINT_ARRAY, zero_wrenches_full_model_2, torque_command_2);
        if (id_solver_result != 0)
        {
            robot_driver_2.stop_robot_motion();
            robot_driver_1.stop_robot_motion();
            printf("Robot stoped: error in dynamics 2\n");
            return;
        }

        if (iteration_count == 1)
        {
            if (robot_driver_1.set_control_mode(control_mode::TORQUE) == -1)
            {
                printf("Incorrect control mode 1\n");
                return;
            }

            if (robot_driver_2.set_control_mode(control_mode::TORQUE) == -1)
            {
                printf("Incorrect control mode 2\n");
                return;
            }
        }

        // Set control commands
        return_flag = robot_driver_1.set_joint_torques(torque_command_1);
        if (return_flag == -1)
        {
            robot_driver_1.stop_robot_motion();
            robot_driver_2.stop_robot_motion();
            printf("Robot stoped: error in control 1\n");
            return;
        }

        return_flag = robot_driver_2.set_joint_torques(torque_command_2);
        if (return_flag == -1)
        {
            robot_driver_2.stop_robot_motion();
            robot_driver_1.stop_robot_motion();
            printf("Robot stoped: error in control 2\n");
            return;
        }

        if (enforce_loop_frequency(DT_MICRO) != 0) control_loop_delay_count++;

        // static double loop_time = 0.0;
        // loop_time += std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - loop_start_time).count();
        // if (iteration_count == 2000) 
        // {
        //     robot_driver.stop_robot_motion();
        //     printf("Average loop time: %f\n", loop_time / 2000.0);
        //     break;
        // }
    }

    robot_driver_1.stop_robot_motion();
    robot_driver_2.stop_robot_motion();
    printf("Task completed\n");
    printf("Control loop delay count: %d\n", control_loop_delay_count);
}

int run_main_control(kinova_mediator &robot_driver_1, kinova_mediator &robot_driver_2)
{
    const double DT_SEC = 1.0 / static_cast<double>(RATE_HZ);
    const int DT_MICRO = SECOND / RATE_HZ;
    const int DT_STOPPING_MICRO = SECOND / dynamics_parameter::STOPPING_MOTION_LOOP_FREQ;

    // Constraint comming from the Vereshchagin HD solver
    assert(JOINTS == robot_driver_1.get_robot_model().getNrOfSegments());
    assert(JOINTS == robot_driver_2.get_robot_model().getNrOfSegments());

    KDL::Chain robot_chain_1 = robot_driver_1.get_robot_model();
    KDL::Chain robot_chain_2 = robot_driver_2.get_robot_model();

    // Above main chain is prepared for vereshchagin (nj == ns) but full contains additional segments
    KDL::Chain robot_chain_full_1 = robot_driver_1.get_full_robot_model();
    KDL::Chain robot_chain_full_2 = robot_driver_2.get_full_robot_model();

    dynamics_controller controller_1(&robot_driver_1, RATE_HZ, compensate_gravity);
    dynamics_controller controller_2(&robot_driver_2, RATE_HZ, compensate_gravity);

    int return_flag = define_task(&controller_1); 
    if (return_flag != 0)
    {
        printf("Error in defining task for robot 1\n");
        return -1;
    }

    return_flag = define_task(&controller_2); 
    if (return_flag != 0)
    {
        printf("Error in defining task for robot 2\n");
        return -1;
    }

    if (desired_task_model == task_model::full_pose) 
    {
        controller_1.set_parameters(time_horizon_amplitude,
                                    max_command, error_alpha,
                                    bias_threshold, bias_step, gain_threshold,
                                    gain_step, min_bias_sat, min_command_sat,
                                    null_space_abag_parameters, compensation_parameters,
                                    STOP_MOTION_ERROR_ALPHA,
                                    STOP_MOTION_BIAS_THRESHOLD, STOP_MOTION_BIAS_STEP,
                                    STOP_MOTION_GAIN_THRESHOLD, STOP_MOTION_GAIN_STEP,
                                    wrench_estimation_gain);

        controller_2.set_parameters(time_horizon_amplitude,
                                    max_command, error_alpha,
                                    bias_threshold, bias_step, gain_threshold,
                                    gain_step, min_bias_sat, min_command_sat,
                                    null_space_abag_parameters, compensation_parameters,
                                    STOP_MOTION_ERROR_ALPHA,
                                    STOP_MOTION_BIAS_THRESHOLD, STOP_MOTION_BIAS_STEP,
                                    STOP_MOTION_GAIN_THRESHOLD, STOP_MOTION_GAIN_STEP,
                                    wrench_estimation_gain);
    }
    else if (desired_task_model == task_model::moveGuarded)
    {
        controller_1.set_parameters(time_horizon_amplitude,
                                    max_command, error_alpha_1,
                                    bias_threshold_1, bias_step_1, gain_threshold_1,
                                    gain_step_1, min_bias_sat, min_command_sat,
                                    null_space_abag_parameters, compensation_parameters,
                                    STOP_MOTION_ERROR_ALPHA,
                                    STOP_MOTION_BIAS_THRESHOLD, STOP_MOTION_BIAS_STEP,
                                    STOP_MOTION_GAIN_THRESHOLD, STOP_MOTION_GAIN_STEP,
                                    wrench_estimation_gain);

        controller_2.set_parameters(time_horizon_amplitude,
                                    max_command, error_alpha_1,
                                    bias_threshold_1, bias_step_1, gain_threshold_1,
                                    gain_step_1, min_bias_sat, min_command_sat,
                                    null_space_abag_parameters, compensation_parameters,
                                    STOP_MOTION_ERROR_ALPHA,
                                    STOP_MOTION_BIAS_THRESHOLD, STOP_MOTION_BIAS_STEP,
                                    STOP_MOTION_GAIN_THRESHOLD, STOP_MOTION_GAIN_STEP,
                                    wrench_estimation_gain);
    }
    else if (desired_task_model == task_model::moveTo_follow_path)
    {
        controller_1.set_parameters(time_horizon_amplitude,
                                    max_command, error_alpha_3,
                                    bias_threshold_3, bias_step_3, gain_threshold_3,
                                    gain_step_3, min_bias_sat, min_command_sat,
                                    null_space_abag_parameters, compensation_parameters,
                                    STOP_MOTION_ERROR_ALPHA,
                                    STOP_MOTION_BIAS_THRESHOLD, STOP_MOTION_BIAS_STEP,
                                    STOP_MOTION_GAIN_THRESHOLD, STOP_MOTION_GAIN_STEP,
                                    wrench_estimation_gain);

        controller_2.set_parameters(time_horizon_amplitude,
                                    max_command, error_alpha_3,
                                    bias_threshold_3, bias_step_3, gain_threshold_3,
                                    gain_step_3, min_bias_sat, min_command_sat,
                                    null_space_abag_parameters, compensation_parameters,
                                    STOP_MOTION_ERROR_ALPHA,
                                    STOP_MOTION_BIAS_THRESHOLD, STOP_MOTION_BIAS_STEP,
                                    STOP_MOTION_GAIN_THRESHOLD, STOP_MOTION_GAIN_STEP,
                                    wrench_estimation_gain);

    }
    else if (desired_task_model == task_model::moveTo_weight_compensation)
    {
        controller_1.set_parameters(time_horizon_amplitude,
                                    max_command, error_alpha_4,
                                    bias_threshold_4, bias_step_4, gain_threshold_4,
                                    gain_step_4, min_bias_sat, min_command_sat,
                                    null_space_abag_parameters, compensation_parameters,
                                    STOP_MOTION_ERROR_ALPHA,
                                    STOP_MOTION_BIAS_THRESHOLD, STOP_MOTION_BIAS_STEP,
                                    STOP_MOTION_GAIN_THRESHOLD, STOP_MOTION_GAIN_STEP,
                                    wrench_estimation_gain);

        controller_2.set_parameters(time_horizon_amplitude,
                                    max_command, error_alpha_4,
                                    bias_threshold_4, bias_step_4, gain_threshold_4,
                                    gain_step_4, min_bias_sat, min_command_sat,
                                    null_space_abag_parameters, compensation_parameters,
                                    STOP_MOTION_ERROR_ALPHA,
                                    STOP_MOTION_BIAS_THRESHOLD, STOP_MOTION_BIAS_STEP,
                                    STOP_MOTION_GAIN_THRESHOLD, STOP_MOTION_GAIN_STEP,
                                    wrench_estimation_gain);
    }
    else if (desired_task_model == task_model::moveConstrained_follow_path)
    {
        controller_1.set_parameters(time_horizon_amplitude,
                                    max_command_moveConstrained, error_alpha_5,
                                    bias_threshold_5, bias_step_5, gain_threshold_5,
                                    gain_step_5, min_bias_sat, min_command_sat,
                                    null_space_abag_parameters_moveConstrained, 
                                    compensation_parameters,
                                    STOP_MOTION_ERROR_ALPHA,
                                    STOP_MOTION_BIAS_THRESHOLD, STOP_MOTION_BIAS_STEP,
                                    STOP_MOTION_GAIN_THRESHOLD, STOP_MOTION_GAIN_STEP,
                                    wrench_estimation_gain);
        controller_2.set_parameters(time_horizon_amplitude,
                                    max_command_moveConstrained, error_alpha_5,
                                    bias_threshold_5, bias_step_5, gain_threshold_5,
                                    gain_step_5, min_bias_sat, min_command_sat,
                                    null_space_abag_parameters_moveConstrained, 
                                    compensation_parameters,
                                    STOP_MOTION_ERROR_ALPHA,
                                    STOP_MOTION_BIAS_THRESHOLD, STOP_MOTION_BIAS_STEP,
                                    STOP_MOTION_GAIN_THRESHOLD, STOP_MOTION_GAIN_STEP,
                                    wrench_estimation_gain);
    }
    else if (desired_task_model == task_model::moveTo)
    {
        controller_1.set_parameters(time_horizon_amplitude, 
                                    max_command, error_alpha_2,
                                    bias_threshold_2, bias_step_2, gain_threshold_2,
                                    gain_step_2, min_bias_sat, min_command_sat,
                                    null_space_abag_parameters, compensation_parameters,
                                    STOP_MOTION_ERROR_ALPHA,
                                    STOP_MOTION_BIAS_THRESHOLD, STOP_MOTION_BIAS_STEP,
                                    STOP_MOTION_GAIN_THRESHOLD, STOP_MOTION_GAIN_STEP,
                                    wrench_estimation_gain);

        controller_2.set_parameters(time_horizon_amplitude, 
                                    max_command, error_alpha_2,
                                    bias_threshold_2, bias_step_2, gain_threshold_2,
                                    gain_step_2, min_bias_sat, min_command_sat,
                                    null_space_abag_parameters, compensation_parameters,
                                    STOP_MOTION_ERROR_ALPHA,
                                    STOP_MOTION_BIAS_THRESHOLD, STOP_MOTION_BIAS_STEP,
                                    STOP_MOTION_GAIN_THRESHOLD, STOP_MOTION_GAIN_STEP,
                                    wrench_estimation_gain);
    }
    else if (desired_task_model == task_model::gravity_compensation)
    {
        controller_1.set_parameters(time_horizon_amplitude,
                                    max_command, error_alpha,
                                    bias_threshold, bias_step, gain_threshold,
                                    gain_step, min_bias_sat, min_command_sat,
                                    null_space_abag_parameters, compensation_parameters,
                                    STOP_MOTION_ERROR_ALPHA,
                                    STOP_MOTION_BIAS_THRESHOLD, STOP_MOTION_BIAS_STEP,
                                    STOP_MOTION_GAIN_THRESHOLD, STOP_MOTION_GAIN_STEP,
                                    wrench_estimation_gain);

        controller_2.set_parameters(time_horizon_amplitude,
                                    max_command, error_alpha,
                                    bias_threshold, bias_step, gain_threshold,
                                    gain_step, min_bias_sat, min_command_sat,
                                    null_space_abag_parameters, compensation_parameters,
                                    STOP_MOTION_ERROR_ALPHA,
                                    STOP_MOTION_BIAS_THRESHOLD, STOP_MOTION_BIAS_STEP,
                                    STOP_MOTION_GAIN_THRESHOLD, STOP_MOTION_GAIN_STEP,
                                    wrench_estimation_gain);
    }
    else
    {
        printf("Error in setting the parameters\n");
        return -1;
    }

    return_flag = controller_1.initialize(desired_control_mode, desired_dynamics_interface, motion_profile_id, log_data_1, use_estimated_external_wrench);
    if (return_flag != 0)
    {
        printf("Error in intializing arm 1\n");
        return -1;
    }

    return_flag = controller_2.initialize(desired_control_mode, desired_dynamics_interface, motion_profile_id, log_data_2, use_estimated_external_wrench);
    if (return_flag != 0)
    {
        printf("Error in intializing arm 2\n");
        return -1;
    }

    KDL::JntArray torque_command_1(7), joint_pos_1(7), joint_vel_1(7), joint_torque_1(7),
                  torque_command_2(7), joint_pos_2(7), joint_vel_2(7), joint_torque_2(7);

    KDL::Wrenches wrenches_full_model_1(robot_chain_full_1.getNrOfSegments(), KDL::Wrench::Zero());
    KDL::Wrenches wrenches_full_model_2(robot_chain_full_2.getNrOfSegments(), KDL::Wrench::Zero());

    KDL::Wrenches wrenches_full_model_sim_1(robot_chain_full_1.getNrOfSegments(), KDL::Wrench::Zero());
    KDL::Wrenches wrenches_full_model_sim_2(robot_chain_full_2.getNrOfSegments(), KDL::Wrench::Zero());

    wrenches_full_model_sim_1[robot_chain_full_1.getNrOfSegments() - 1] = KDL::Wrench(KDL::Vector(0.0, -2.0, 0.0),
                                                                                      KDL::Vector(0.0, 0.0, 0.0));

    wrenches_full_model_sim_2[robot_chain_full_2.getNrOfSegments() - 1] = KDL::Wrench(KDL::Vector(0.0, -2.0, 0.0),
                                                                                      KDL::Vector(0.0, 0.0, 0.0));

    double total_time_sec = 0.0;
    int loop_iteration_count = 0;
    int stop_loop_iteration_count = 0;
    int control_loop_delay_count = 0;
    bool stopping_sequence_on = false;
    bool trigger_stopping_sequence = false;
    bool robot_1_locked = false;
    bool robot_2_locked = false;
    int return_flag_1 = 0;
    int return_flag_2 = 0;

    int estimation_flag_1 = 0;
    int estimation_flag_2 = 0;

    // Real-time loop
    while (1)
    {
        // Save current time point
        loop_start_time = std::chrono::steady_clock::now();
        if (!stopping_sequence_on) total_time_sec = loop_iteration_count * DT_SEC;

        // Get current state from robot sensors
        if (use_estimated_external_wrench && !stopping_sequence_on)
        {
            robot_driver_1.get_joint_state(joint_pos_1, joint_vel_1, joint_torque_1);
            robot_driver_2.get_joint_state(joint_pos_2, joint_vel_2, joint_torque_2);

            estimation_flag_1 = controller_1.estimate_external_wrench(joint_pos_1, joint_vel_1, joint_torque_1, wrenches_full_model_1[robot_chain_full_1.getNrOfSegments()- 1]);
            if (estimation_flag_1 != 0)
            {
                printf("Error in external wrench estimation, robot 1\n");
                trigger_stopping_sequence = true;
            }

            estimation_flag_2 = controller_2.estimate_external_wrench(joint_pos_2, joint_vel_2, joint_torque_2, wrenches_full_model_2[robot_chain_full_2.getNrOfSegments()- 1]);
            if (estimation_flag_2 != 0)
            {
                printf("Error in external wrench estimation, robot 2\n");
                trigger_stopping_sequence = true;
            }
        }
        else
        {
            robot_driver_1.get_robot_state(joint_pos_1, joint_vel_1, joint_torque_1, wrenches_full_model_1[robot_chain_full_1.getNrOfSegments() - 1]);
            robot_driver_2.get_robot_state(joint_pos_2, joint_vel_2, joint_torque_2, wrenches_full_model_2[robot_chain_full_2.getNrOfSegments() - 1]);
        }

        // Make one control iteration (step) for both robots -> Update control commands
        if (!robot_1_locked) return_flag_1 = controller_1.step(joint_pos_1, joint_vel_1, joint_torque_1, wrenches_full_model_1[robot_chain_full_1.getNrOfSegments()- 1], torque_command_1, total_time_sec, loop_iteration_count, stop_loop_iteration_count, stopping_sequence_on);
        if (return_flag_1 == -1) trigger_stopping_sequence = true;

        if (!trigger_stopping_sequence && !robot_2_locked) return_flag_2 = controller_2.step(joint_pos_2, joint_vel_2, joint_torque_2, wrenches_full_model_2[robot_chain_full_2.getNrOfSegments()- 1], torque_command_2, total_time_sec, loop_iteration_count, stop_loop_iteration_count, stopping_sequence_on);
        if (return_flag_2 == -1) trigger_stopping_sequence = true;

        if (stopping_sequence_on) // Robots will be controlled to stop their motion and eventually lock
        {
            if (return_flag_1 == 1 && !robot_1_locked) // Stop motion task completed for robot 1
            {
                // Make sure that the robot is locked (freezed)
                controller_1.engage_lock();
                robot_1_locked = true;
            }

            if (return_flag_2 == 1 && !robot_2_locked) // Stop motion task completed for robot 2
            {
                // Make sure that the robot is locked (freezed)
                controller_2.engage_lock();
                robot_2_locked = true;
            }

            if (robot_1_locked && robot_2_locked)
            {
                stopping_sequence_on = false;
                total_time_sec += (double)stop_loop_iteration_count / dynamics_parameter::STOPPING_MOTION_LOOP_FREQ;
                printf("Robots locked!\n");
                controller_1.deinitialize();
                controller_2.deinitialize();
                return 0;                
            }

            if (!robot_1_locked)
            {
                // Apply robot 1 joint commands using torque control interface (bypass all safety checks)
                if (controller_1.apply_joint_control_commands(true) == -1)
                {
                    // Make sure that the robot is locked (freezed)
                    controller_1.engage_lock();
                    robot_1_locked = true;
                }
            }

            if (!robot_2_locked)
            {
                // Apply robot 2 joint commands using torque control interface (bypass all safety checks)
                if (controller_2.apply_joint_control_commands(true) == -1)
                {
                    // Make sure that the robot is locked (freezed)
                    controller_2.engage_lock();
                    robot_2_locked = true;
                }
            }

            stop_loop_iteration_count++;
            if (enforce_loop_frequency(DT_STOPPING_MICRO) != 0) control_loop_delay_count++;

            // Testing loop time
            // static double loop_time = 0.0;
            // loop_time += std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - loop_start_time).count();
            // if (stop_loop_iteration_count == 500) 
            // {
            //     robot_driver_1.stop_robot_motion();
            //     robot_driver_2.stop_robot_motion();
            //     printf("Stop loop time: %f\n", loop_time / 500.0);
            //     controller_1.deinitialize();
            //     controller_2.deinitialize();
            //     return 0;
            // }
        }
        else // Nominal task execution mode
        {
            // Set external wrenches for the simulation. FD solver expects wrenches to be expressed in respective link's frame... not the base frame
            if (loop_iteration_count == 3000) robot_driver_1.set_ext_wrenches_sim(wrenches_full_model_sim_1);
            if (loop_iteration_count == 3000) robot_driver_2.set_ext_wrenches_sim(wrenches_full_model_sim_2);

            if (!trigger_stopping_sequence)
            {
                // Apply joint commands using safe control interface
                if (controller_1.apply_joint_control_commands(false) == -1) trigger_stopping_sequence = true;
                if (controller_2.apply_joint_control_commands(false) == -1) trigger_stopping_sequence = true;
            }

            if (trigger_stopping_sequence)
            {
                trigger_stopping_sequence = false;
                stopping_sequence_on = true;
                stop_loop_iteration_count = 0;

                printf("Stopping behaviour triggered!\n");
                continue;
            }

            loop_iteration_count++;
            if (enforce_loop_frequency(DT_MICRO) != 0) control_loop_delay_count++;

            // Testing loop time
            // static double loop_time = 0.0;
            // loop_time += std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - loop_start_time).count();
            // if (loop_iteration_count == 2000) 
            // {
            //     printf("Main loop time: %f\n", loop_time / 2000.0);
            //     printf("Control loop delay count: %d\n", control_loop_delay_count);
            //     trigger_stopping_sequence = true;
            // }
        }
    }

    robot_driver_1.stop_robot_motion();
    robot_driver_2.stop_robot_motion();
    controller_1.deinitialize();
    controller_2.deinitialize();
    printf("Task completed\n");
    return 0;
}

int main(int argc, char **argv)
{
    RATE_HZ              = 700; // Hz
    control_dims         = std::vector<bool>{true, true, true, // Linear
                                             false, false, false}; // Angular
                                            //  true, true, true}; // Angular
    control_dims_moveConstrained = {true, true, true, // Linear
                                    true, true, false}; // Angular

    tube_tolerances      = std::vector<double>{0.01, 0.01, 0.01,
                                               0.01, 0.0, 0.0,
                                               0.0, 0.0}; // Last tolerance is in unit of degrees - Null-space tolerance
    // Tube tolerances: x pos,    y pos,      z force,
    //                  x torque, y torque,   null-space, 
    //                  x vel,    z_a pos/vel
    tube_tolerances_moveConstrained = std::vector<double> {0.003, 0.005, 0.003,
                                                           0.0, 0.0, 25.0,
                                                           0.0, 0.005};

    max_command                     = (Eigen::VectorXd(NUMBER_OF_CONSTRAINTS) << 20.0, 20.0, 20.0, 20.0, 20.0, 20.0).finished();
    max_command_moveConstrained     = (Eigen::VectorXd(NUMBER_OF_CONSTRAINTS) << 20.0, 20.0, 50.0, 9.5, 9.5, 20.0).finished();
    path_parameters                 = std::vector<double>{0.5, 0.5, 0.05, 0.008, 2}; // last parameter must be min 2 (number of points)
    path_parameters_moveConstrained = std::vector<double>{1.5, 1.7, 0.03, 0.003, 100};
    environment          = kinova_environment::REAL;
    robot_model_id       = kinova_model::URDF;
    desired_pose_id      = desired_pose::APPROACH_TABLE;
    desired_control_mode = control_mode::TORQUE;
    desired_task_model   = task_model::moveConstrained_follow_path;
    path_type            = path_types::INF_SIGN_PATH;
    motion_profile_id    = m_profile::CONSTANT;
    time_horizon_amplitude = 2.5;
    task_time_limit_sec  = 80.5;
    tube_speed           = 0.02;
    tube_force           = -12.5;
    contact_threshold_linear  = 50.0;
    contact_threshold_angular = 50.0;
    compensate_gravity   = true;
    control_null_space   = false;
    use_mass_alternation = false;
    log_data_1           = true;
    log_data_2           = false;
    use_estimated_external_wrench  = true;
    control_null_space_moveConstrained = false;

    kinova_mediator robot_driver_1;
    kinova_mediator robot_driver_2;
    int return_flag = go_to(robot_driver_1, robot_driver_2, desired_pose_id);
    if (return_flag != 0) return 0;

    // Extract robot model and if not simulation, establish connection with motor drivers
    if (!robot_driver_1.is_initialized()) robot_driver_1.initialize(robot_model_id, environment, robot_id::KINOVA_GEN3_1, 1.0 / static_cast<double>(RATE_HZ));
    if (!robot_driver_1.is_initialized())
    {
        printf("Robot 1 is not initialized\n");
        return 0;
    }

    if (!robot_driver_2.is_initialized()) robot_driver_2.initialize(robot_model_id, environment, robot_id::KINOVA_GEN3_2, 1.0 / static_cast<double>(RATE_HZ));
    if (!robot_driver_2.is_initialized())
    {
        printf("Robot 2 is not initialized\n");
        return 0;
    }

    // run_test(robot_driver_1, robot_driver_2); return 0;

    if (run_main_control(robot_driver_1, robot_driver_2) == -1) return 0;

    robot_driver_1.deinitialize();
    robot_driver_2.deinitialize();

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    return_flag = go_to(robot_driver_1, robot_driver_2, desired_pose::APPROACH_TABLE);
    return 0;
}