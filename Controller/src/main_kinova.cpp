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
#include <external_wrench_estimator.hpp>
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
    HOME_DOWN      = 6,
    HOME_UP        = 7,
    HOME_UP_2      = 8,
    APPROACH_TABLE = 9
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
int id                               = robot_id::KINOVA_GEN3_1;
double DT_SEC                        = 1.0 / static_cast<double>(RATE_HZ);
double time_horizon_amplitude        = 2.5;
double tube_speed                    = 0.01;
double tube_force                    = 0.03;
double desired_null_space_angle      = 90.0; // Unit degrees
double task_time_limit_sec           = 600.0;
double contact_threshold_linear      = 35.0; // N
double contact_threshold_angular     = 2.0; // Nm

bool log_data                        = false;
bool use_estimated_external_wrench   = false;
bool control_null_space              = false;
bool control_null_space_moveConstrained = false;
bool compensate_gravity              = false;
bool use_mass_alternation            = false;
auto error_callback = [](Kinova::Api::KError err){ cout << "_________ callback error _________" << err.toString(); };

std::vector<bool> control_dims                 = {true, true, true, // Linear
                                                  false, false, false}; // Angular
std::vector<bool> control_dims_moveConstrained = {true, true, true, // Linear
                                                  true, true, false}; // Angular

std::vector<double> path_parameters       = {1.5, 1.7, 0.03, 0.003, 2}; // last parameter must be min 2 (number of points)
std::vector<double> path_parameters_moveConstrained = {1.5, 1.7, 0.03, 0.003, 100};
std::vector<double> tube_start_position   = {0.0, 0.0, 0.0};
std::vector<double> tube_tolerances       = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

// Tube tolerances: x pos,    y pos,      z force, 
//                  x torque, y torque,   null-space, 
//                  x vel,    z_a pos/vel
std::vector<double> tube_tolerances_moveConstrained = {0.003, 0.03, 0.003,
                                                       0.005, 0.005, 25.0,
                                                       0.003, 0.001};

std::vector< std::vector<double> > tube_path_points(path_parameters[4], std::vector<double>(3, 0.0));
std::vector< std::vector<double> > tube_path_points_moveConstrained(path_parameters_moveConstrained[4], std::vector<double>(3, 0.0));
std::vector< std::vector<double> > path_poses(path_parameters[4] - 1,   std::vector<double>(12, 0.0));
std::vector< std::vector<double> > path_poses_moveConstrained(path_parameters_moveConstrained[4] - 1,   std::vector<double>(12, 0.0));

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
                                               0.000250, 0.000250, 0.000250).finished();
const Eigen::VectorXd bias_step_2           = (Eigen::VectorXd(NUMBER_OF_CONSTRAINTS) \
                                            << 0.000500, 0.000400, 0.000400, 
                                               0.000900, 0.000900, 0.000900).finished();
const Eigen::VectorXd gain_threshold_2      = (Eigen::VectorXd(NUMBER_OF_CONSTRAINTS) \
                                            << 0.502492, 0.502492, 0.502492, 
                                               0.500000, 0.500000, 0.500000).finished();
const Eigen::VectorXd gain_step_2           = (Eigen::VectorXd(NUMBER_OF_CONSTRAINTS) \
                                            << 0.002552, 0.002552, 0.002552, 
                                               0.001000, 0.001000, 0.001000).finished();

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

// Define the callback function used in Refresh_callback
auto lambda_fct_callback = [](const Kinova::Api::Error &err, const Kinova::Api::BaseCyclic::Feedback data)
{
    // We are printing the data of the moving actuator just for the example purpose,
    // avoid this in a real-time loop
    std::string serialized_data;
    google::protobuf::util::MessageToJsonString(data.actuators(6), &serialized_data);
    std::cout << serialized_data << std::endl << std::endl;
};


void rotate_joint(kinova_mediator &robot_driver, const int joint, const double rate)
{
    robot_driver.set_control_mode(control_mode::VELOCITY);
    KDL::JntArray rotate_joint(JOINTS);
    rotate_joint(joint) = rate; 
    robot_driver.set_joint_velocities(rotate_joint);
    if (environment != kinova_environment::SIMULATION) usleep(5000 * MILLISECOND);
}

/**
 * Code for calibrating torque offets. 
 * Should be used only one-time. The robot must be in the zero-configuration.
 */
void calibrate_torque_offsets()
{
    // Extract user password from a file
    fstream newfile;
    string kinova_passwd;
    newfile.open("/home/djole/Master/Thesis/GIT/MT_testing/kinova_passwd.txt", ios::in);
    if (newfile.is_open())
    {
        if (id == KINOVA_GEN3_1) getline(newfile, kinova_passwd);
        else while (getline(newfile, kinova_passwd)) {}
        newfile.close();
    }

    // Create API objects
    auto transport = new Kinova::Api::TransportClientTcp();
    auto router = new Kinova::Api::RouterClient(transport, error_callback);
    if (id == KINOVA_GEN3_1) transport->connect(IP_ADDRESS_1, PORT);
    else transport->connect(IP_ADDRESS_2, PORT);

    // Set session data connection information
    auto create_session_info = Kinova::Api::Session::CreateSessionInfo();
    create_session_info.set_username("admin");
    create_session_info.set_password(kinova_passwd);
    create_session_info.set_session_inactivity_timeout(200);   // (milliseconds)
    create_session_info.set_connection_inactivity_timeout(200); // (milliseconds)

    // Session manager service wrapper
    auto session_manager = new Kinova::Api::SessionManager(router);
    session_manager->CreateSession(create_session_info);

    // Create services
    auto base = new Kinova::Api::Base::BaseClient(router);
    auto actuator_config = new Kinova::Api::ActuatorConfig::ActuatorConfigClient(router);

    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    auto torque_offset_message = Kinova::Api::ActuatorConfig::TorqueOffset();
    torque_offset_message.set_torque_offset(0.0f);
    // torque_offset_message.set_torque_offset(0.00592135f);
    actuator_config->SetTorqueOffset(torque_offset_message, 1);

    torque_offset_message = Kinova::Api::ActuatorConfig::TorqueOffset();
    // torque_offset_message.set_torque_offset(0.0f);
    torque_offset_message.set_torque_offset(0.00253617f);
    // torque_offset_message.set_torque_offset(-0.165179f);
    actuator_config->SetTorqueOffset(torque_offset_message, 2);

    torque_offset_message = Kinova::Api::ActuatorConfig::TorqueOffset();
    torque_offset_message.set_torque_offset(0.0f);
    // torque_offset_message.set_torque_offset(0.00214829f);
    actuator_config->SetTorqueOffset(torque_offset_message, 3);

    torque_offset_message = Kinova::Api::ActuatorConfig::TorqueOffset();
    // torque_offset_message.set_torque_offset(0.0f);
    torque_offset_message.set_torque_offset(0.00153136f);
    // torque_offset_message.set_torque_offset(-0.0492138f);
    actuator_config->SetTorqueOffset(torque_offset_message, 4);

    torque_offset_message = Kinova::Api::ActuatorConfig::TorqueOffset();
    torque_offset_message.set_torque_offset(0.0f);
    // torque_offset_message.set_torque_offset(-0.00036302f);
    actuator_config->SetTorqueOffset(torque_offset_message, 5);

    torque_offset_message = Kinova::Api::ActuatorConfig::TorqueOffset();
    // torque_offset_message.set_torque_offset(0.0f);
    torque_offset_message.set_torque_offset(0.00137371f);
    // torque_offset_message.set_torque_offset(-0.00558801f);
    actuator_config->SetTorqueOffset(torque_offset_message, 6);

    torque_offset_message = Kinova::Api::ActuatorConfig::TorqueOffset();
    torque_offset_message.set_torque_offset(0.0f);
    // torque_offset_message.set_torque_offset(-0.000393912f);
    actuator_config->SetTorqueOffset(torque_offset_message, 7);

    std::this_thread::sleep_for(std::chrono::milliseconds(1500));

    auto control_mode = actuator_config->GetControlMode(6);
    std::cout << "ControlMode: " << control_mode.control_mode() << std::endl; 
    auto torque_offset = actuator_config->GetTorqueOffset(6);
    std::cout << "TorqueOffset: " << torque_offset.torque_offset() << std::endl;
    
    // Close API session
    session_manager->CloseSession();

    // Deactivate the router and cleanly disconnect from the transport object
    router->SetActivationStatus(false);
    transport->disconnect();

    // Destroy the API
    delete actuator_config;
    delete base;
    delete session_manager;
    delete router;
    delete transport;

    // printf("Torque offset calibration completed.\n");
}

int go_to(kinova_mediator &robot_driver, const int desired_pose_)
{
    std::vector<double> configuration_array(7, 0.0);
    switch (desired_pose_) // Angle value are in units of degree
    {
        case desired_pose::CANDLE:
            configuration_array = std::vector<double> {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            break;
        case desired_pose::APPROACH_TABLE:
            // configuration_array = std::vector<double> {0.001, 42.017, 179.56, 220.641, 2.761, 1.965, 88.101}; // for test with out polishing tool
            // configuration_array = std::vector<double> {48.795, 67.916, 76.724, 241.199, 64.802, 270.236, 18.503};// for test with polishing tool:center
            // configuration_array = std::vector<double> {1.973, 31.26, 176.732, 267.612, 1.51, 302.826, 89.919};// for test with polishing tool:front
            // configuration_array = std::vector<double> {224.329, 102.519, 118.864, 246.859, 301.916, 68.074, 190.767};// for test with polishing tool:180 deg back and down
            configuration_array = std::vector<double> {0.0, 29.802, 180.487, 262.51, 0.0, 312.097, 90.287};// for test with polishing tool:front, ramp 180deg inverted
            // configuration_array = std::vector<double> {350.32, 51.578, 236.031, 290.273, 315.925, 284.194, 133.585};// for test with polishing tool:front, ramp 180deg inverted, null_space moved
            break;
        case desired_pose::HOME_BACK:
            configuration_array = std::vector<double> {180.0, 15.0, 180.0, 230.0, 0.0, 55.0, 90.0};
            break;
        case desired_pose::PACKAGING:
            configuration_array = std::vector<double> {0.0, 330.0, 180.0, 214.0, 0.0, 115.0, 270.0};
            break;
        case desired_pose::RETRACT:
            configuration_array = std::vector<double> {0.0, 340.0, 180.0, 214.0, 0.0, 310.0, 90.0};
            break;
        case desired_pose::HOME_DOWN:
            configuration_array = std::vector<double> {353.962, 2.714, 185.847, 260.143, 1.81, 327.854, 88.317};
            break;
        case desired_pose::HOME_UP:
            configuration_array = std::vector<double> {354.009, 5.012, 187.623, 237.876, 1.868, 66.414, 87.982};
            break;
        case desired_pose::HOME_UP_2:
            configuration_array = std::vector<double> {359.793, 97.913, 4.262, 238.062, 2.122, 74.802, 266.611};
            break;
        case desired_pose::HOME:
            configuration_array = std::vector<double> {0.0, 15.0, 180.0, 230.0, 0.0, 55.0, 90.0};
            break;
        default: return -1;
    }

    if (environment != kinova_environment::SIMULATION)
    {
        // Extract user password from a file
        fstream newfile;
        string kinova_passwd;
        newfile.open("/home/djole/Master/Thesis/GIT/MT_testing/kinova_passwd.txt", ios::in);
        if (newfile.is_open())
        {
            if (id == KINOVA_GEN3_1) getline(newfile, kinova_passwd);
            else while (getline(newfile, kinova_passwd)) {}
            newfile.close();
        }

        // Create API objects
        auto transport = new Kinova::Api::TransportClientTcp();
        auto router = new Kinova::Api::RouterClient(transport, error_callback);
        if (id == KINOVA_GEN3_1) transport->connect(IP_ADDRESS_1, PORT);
        else transport->connect(IP_ADDRESS_2, PORT);

        // Set session data connection information
        auto create_session_info = Kinova::Api::Session::CreateSessionInfo();
        create_session_info.set_username("admin");
        create_session_info.set_password(kinova_passwd);
        create_session_info.set_session_inactivity_timeout(200);   // (milliseconds)
        create_session_info.set_connection_inactivity_timeout(200); // (milliseconds)

        // Session manager service wrapper
        // std::cout << "Creating sessions for communication" << std::endl;
        auto session_manager = new Kinova::Api::SessionManager(router);
        session_manager->CreateSession(create_session_info);
        // std::cout << "Sessions created" << std::endl;

        // Create services
        auto base = new Kinova::Api::Base::BaseClient(router);

        // Make sure the arm is in Single Level Servoing before executing an Action
        auto servoingMode = Kinova::Api::Base::ServoingModeInformation();
        servoingMode.set_servoing_mode(Kinova::Api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
        base->SetServoingMode(servoingMode);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        auto constrained_joint_angles = Kinova::Api::Base::ConstrainedJointAngles();
        auto joint_angles = constrained_joint_angles.mutable_joint_angles();
        auto actuator_count = base->GetActuatorCount();

        for (size_t i = 0; i < actuator_count.count(); ++i) 
        {
            auto joint_angle = joint_angles->add_joint_angles();
            joint_angle->set_joint_identifier(i);
            joint_angle->set_value(configuration_array[i]);
        }

        // Connect to notification action topic (Promise alternative)
        // See cartesian examples for Reference alternative
        std::promise<Kinova::Api::Base::ActionEvent> finish_promise;
        auto finish_future = finish_promise.get_future();
        auto promise_notification_handle = base->OnNotificationActionTopic(
            create_event_listener_by_promise(finish_promise),
            Kinova::Api::Common::NotificationOptions()
        );

        // std::cout << "Reaching joint angles..." << std::endl;
        base->PlayJointTrajectory(constrained_joint_angles);

        // Wait for future value from promise (Promise alternative)
        // See cartesian examples for Reference alternative
        const auto status = finish_future.wait_for(TIMEOUT_DURATION);
        base->Unsubscribe(promise_notification_handle);

        if (status != std::future_status::ready)
        {
            std::cout << "Timeout on action notification wait" << std::endl;
            std::cout << "Can't reach safe position, exiting" << std::endl;

            // Close API session
            session_manager->CloseSession();

            // Deactivate the router and cleanly disconnect from the transport object
            router->SetActivationStatus(false);
            transport->disconnect();

            // Destroy the API
            delete base;
            delete session_manager;
            delete router;
            delete transport;
            return -1;
        }

        // const auto promise_event = finish_future.get();

        // std::cout << "Joint angles reached" << std::endl;
        // std::cout << "Promise value : " << Kinova::Api::Base::ActionEvent_Name(promise_event) << std::endl;

        // Close API session
        session_manager->CloseSession();

        // Deactivate the router and cleanly disconnect from the transport object
        router->SetActivationStatus(false);
        transport->disconnect();

        // Destroy the API
        delete base;
        delete session_manager;
        delete router;
        delete transport;
        // printf("High-Level Control Completed\n");
    }
    else
    {
        robot_driver.initialize(robot_model_id, environment, id, 1.0 / static_cast<double>(RATE_HZ));
        if (!robot_driver.is_initialized())
        {
            printf("Robot is not initialized\n");
            return -1;
        }

        if (robot_driver.set_control_mode(control_mode::POSITION) == -1)
        {
            printf("Incorrect control mode\n");
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

        robot_driver.set_joint_positions(config);
    }
    return 0;
}

int define_task(dynamics_controller *dyn_controller)
{
    std::vector<double> desired_ee_pose(12, 0.0);
    switch (desired_pose_id)
    {
        case desired_pose::CANDLE:
            tube_start_position = std::vector<double>{0.0, -0.0248591, 1.12586};
            desired_ee_pose     = { 0.0, -0.0248591, 1.12586, // Linear: Vector
                                    1.0, 0.0, 0.0, // Angular: Rotation matrix
                                    0.0, -1.0, 0.0,
                                    0.0, 0.0, -1.0};
            break;

        case desired_pose::APPROACH_TABLE:
            // tube_start_position = std::vector<double>{0.275073, 0.00364068, 0.177293}; // for test with out polishing tool
            // desired_ee_pose     = { 0.275073, 0.00364068, 0.177293, // Linear: Vector
            //                         -0.0207239, -0.999733, -0.010216, // Angular: Rotation matrix
            //                         0.999763, -0.0206545, -0.00685178,
            //                         0.00663895, -0.0103556, 0.999924};

            // tube_start_position = std::vector<double>{0.336987, 0.00144331, 0.350254}; // for test with polishing tool:center
            // desired_ee_pose     = {0.336987, 0.00144331, 0.350254, // Linear: Vector
            //                        -0.0206334, -0.999743, -0.00941167, // Angular: Rotation matrix
            //                         0.999764, -0.0205681, -0.0069796,
            //                         0.00678422, -0.00955347, 0.999931};

            // tube_start_position = std::vector<double>{0.477894, 0.00349779, 0.365029};// for test with polishing tool:front
            // desired_ee_pose     = {0.477894, 0.00349779, 0.365029, // Linear: Vector
            //                        -0.0275078, -0.99953, 0.0135681, // Angular: Rotation matrix
            //                         0.999593, -0.0276079, -0.00724534,
            //                         0.00761651, 0.0133633, 0.999882};

            // tube_start_position = std::vector<double>{-0.374525, 0.00627562, -0.00995563};// for test with polishing tool:180 deg back and down
            // desired_ee_pose     = {-0.374525, 0.00627562, -0.00995563, // Linear: Vector
            //                        0.000183558, 0.991793, 0.127856, // Angular: Rotation matrix
            //                        -1, 0.000178096, 5.41554e-05,
            //                        3.09403e-05, -0.127856, 0.991793};

            tube_start_position = std::vector<double>{0.468169, -0.00181314, 0.353854};// for test with polishing tool:front, ramp 180 deg inverted
            desired_ee_pose     = {0.468169, -0.00181314, 0.353854, // Linear: Vector
                                   0.00238073, -0.996484, -0.083747, // Angular: Rotation matrix
                                   0.999986, 0.0019835, 0.00482606,
                                   -0.00464298, -0.0837573, 0.996475};
            // tube_start_position = std::vector<double>{0.565726, -0.1577, 0.371041};// for test with polishing tool:front, ramp 180 deg inverted, null_space moved
            // desired_ee_pose     = {0.565726, -0.1577, 0.371041, // Linear: Vector
            //                        -0.00325582, -0.997711, -0.0675378, // Angular: Rotation matrix
            //                        0.99999, -0.00345493, 0.00283143,
                                    // -0.00305829, -0.0675279, 0.997713};
            break;

        case desired_pose::RETRACT:
            tube_start_position = std::vector<double>{0.117804, 0.00134572, 0.489747};
            desired_ee_pose     = { 0.117804, 0.00134572, 0.389747, // Linear: Vector
                                    0.0, -0.997564, -0.0697565, // Angular: Rotation matrix
                                    1.0,  0.0,       0.0,
                                    0.0, -0.0697565, 0.997564};
            break;

        case desired_pose::HOME_FORWARD:
            tube_start_position = std::vector<double>{0.39514, 0.00134662, 0.433724};
            desired_ee_pose     = { 0.39514, 0.00134662, 0.433724, // Linear: Vector
                                    0.0, 0.0, -1.0, // Angular: Rotation matrix
                                    1.0, 0.0, 0.0,
                                    0.0, -1.0, 0.0};
            break;

        case desired_pose::HOME_BACK:
            tube_start_position = std::vector<double>{-0.465034, -0.0131987, 0.149422};
            desired_ee_pose     = {  -0.465034, -0.0131987, 0.149422, // Linear: Vector
                                      0.0307943, 0.998958, -0.0336904, // Angular: Rotation matrix
                                     -0.999509, 0.0309717, 0.00475512,
                                      0.00579361, 0.0335274, 0.999421};
            break;

        case desired_pose::HOME_DOWN:
            tube_start_position = std::vector<double>{ 0.403155, 0.00236782, 0.562052};
            desired_ee_pose     = { 0.53555, 0.00236782, 0.410052, // Linear: Vector
                                    0.0, -0.819152, -1.0, // Angular: Rotation matrix
                                    1.0,  0.0, 0.0,
                                    0.0, -0.573576, 0.819152};
            break;

        case desired_pose::HOME_UP:
            tube_start_position = std::vector<double>{0.382323, -0.00355718, 0.547452};
            desired_ee_pose     = { 0.552323, -0.00355718, 0.647452, // Linear: Vector
                                    0.0, -0.819152, -1.0, // Angular: Rotation matrix
                                    1.0,  0.0, 0.0,
                                    0.0, -0.573576, 0.819152};
            break;

        case desired_pose::HOME_UP_2:
            tube_start_position = std::vector<double>{0.382323, -0.00355718, 0.547452};
            desired_ee_pose     = { 0.552323, -0.00355718, 0.657452, // Linear: Vector
                                    0.0, -0.819152, -1.0, // Angular: Rotation matrix
                                    1.0,  0.0, 0.0,
                                    0.0, -0.573576, 0.819152};
            break;

        case desired_pose::HOME:
            tube_start_position = std::vector<double>{0.395153, 0.0013505, 0.433652};
            desired_ee_pose     = { 0.565153, 0.0013505, 0.433652, // Linear: Vector
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
                    motion_profile::draw_inf_sign_xy(tube_path_points_moveConstrained, 0.25, 0.25, 0.4, 1.0, desired_ee_pose[0], desired_ee_pose[1], desired_ee_pose[2]);
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
    }

    return 0;
}

void run_test(kinova_mediator &robot_driver)
{
    const int DT_MICRO = SECOND / RATE_HZ;
    double total_time_sec = 0.0;
    int iteration_count = 0;
    int control_loop_delay_count = 0;
    int return_flag = 0;

    KDL::JntArray jnt_command_torque(7), jnt_position(7), jnt_velocity(7), jnt_torque(7);
    KDL::Wrench end_effector_wrench;
    const KDL::JntArray ZERO_JOINT_ARRAY(7);

    KDL::Chain robot_chain = robot_driver.get_robot_model();
    // Above main chain is prepared for vereshchagin (nj == ns) but full contains additional segments
    KDL::Chain robot_chain_full = robot_driver.get_full_robot_model();

    const KDL::Wrenches zero_wrenches_full_model(robot_chain_full.getNrOfSegments(), KDL::Wrench::Zero());

    std::shared_ptr<KDL::Solver_RNE> id_solver = std::make_shared<KDL::Solver_RNE>(robot_chain_full, -1 * robot_driver.get_root_acceleration().vel, robot_driver.get_joint_inertia(), robot_driver.get_joint_torque_limits(), true);

    // Real-time loop
    while (total_time_sec < task_time_limit_sec)
    {
        loop_start_time = std::chrono::steady_clock::now();
        iteration_count++;
        total_time_sec = iteration_count * DT_SEC;

        // robot_driver.get_robot_state(jnt_position, jnt_velocity, jnt_torque, end_effector_wrench);
        robot_driver.get_joint_state(jnt_position, jnt_velocity, jnt_torque);

        return_flag = id_solver->CartToJnt(jnt_position, ZERO_JOINT_ARRAY, ZERO_JOINT_ARRAY, zero_wrenches_full_model, jnt_command_torque);
        if (return_flag != 0)
        {
            robot_driver.stop_robot_motion();
            printf("Robot stoped: error in dynamics %d\n", return_flag);
            return;
        }

        if (iteration_count == 1)
        {
            if (robot_driver.set_control_mode(control_mode::TORQUE) == -1)
            {
                printf("Problem with setting control mode in robot's mediator\n");
                return;
            }
        }

        return_flag = robot_driver.set_joint_torques(jnt_command_torque);
        if (return_flag == -1)
        {
            robot_driver.stop_robot_motion();
            printf("Robot stoped: error in control\n");
            return;
        }

        if (enforce_loop_frequency(DT_MICRO) != 0) control_loop_delay_count++;

        // static double loop_time = 0.0;
        // loop_time += std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - loop_start_time).count();
        // if (iteration_count == 10000) 
        // {
        //     robot_driver.stop_robot_motion();
        //     printf("Average loop time: %f\n", loop_time / 10000.0);
        //     break;
        // }
    }

    robot_driver.stop_robot_motion();
    printf("Task completed\n");
    printf("Control loop delay count: %d\n", control_loop_delay_count);
}

int run_main_control(kinova_mediator &robot_driver)
{
    const int DT_MICRO = SECOND / RATE_HZ;
    const int DT_STOPPING_MICRO = SECOND / dynamics_parameter::STOPPING_MOTION_LOOP_FREQ;

    // Constraint comming from the Vereshchagin HD solver
    assert(JOINTS == robot_driver.get_robot_model().getNrOfSegments());

    KDL::Chain robot_chain = robot_driver.get_robot_model();
    // Above main chain is prepared for vereshchagin (nj == ns) but full contains additional segments
    KDL::Chain robot_chain_full = robot_driver.get_full_robot_model();

    dynamics_controller controller(&robot_driver, RATE_HZ, compensate_gravity);
    KDL::ChainExternalWrenchEstimator ext_wrench_estimator(robot_chain_full, -1 * robot_driver.get_root_acceleration().vel, robot_driver.get_joint_inertia(), RATE_HZ, 30.0, 0.5);

    int return_flag = define_task(&controller);
    if (return_flag != 0)
    {
        printf("Error in defining task for robot\n");
        return -1;
    }

    if (desired_task_model == task_model::full_pose) 
    {
        controller.set_parameters(time_horizon_amplitude,
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
        controller.set_parameters(time_horizon_amplitude,
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
        controller.set_parameters(time_horizon_amplitude,
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
        controller.set_parameters(time_horizon_amplitude,
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
        controller.set_parameters(time_horizon_amplitude, 
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
        controller.set_parameters(time_horizon_amplitude,
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
        controller.set_parameters(time_horizon_amplitude,
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

    return_flag = controller.initialize(desired_control_mode, desired_dynamics_interface, motion_profile_id, log_data, use_estimated_external_wrench);
    if (return_flag != 0)
    {
        printf("Error in intializing the arm\n");
        return -1;
    }

    KDL::JntArray torque_command(7), joint_pos(7), joint_vel(7), joint_torque(7);
    KDL::Wrenches wrenches_full_model(robot_chain_full.getNrOfSegments(), KDL::Wrench::Zero());
    KDL::Wrenches wrenches_full_model_sim(robot_chain_full.getNrOfSegments(), KDL::Wrench::Zero());

    // FD solver expects wrenches to be expressed in respective link's frame... not the base frame
    wrenches_full_model_sim[robot_chain_full.getNrOfSegments() - 1] = KDL::Wrench(KDL::Vector(1.0, -2.2, 0.0),
                                                                                  KDL::Vector(0.0,  3.0, 0.0));

    double total_time_sec = 0.0;
    int loop_iteration_count = 0;
    int stop_loop_iteration_count = 0;
    int control_loop_delay_count = 0;
    bool stopping_sequence_on = false;
    bool trigger_stopping_sequence = false;
    return_flag = 0;

    robot_driver.get_joint_state(joint_pos, joint_vel, joint_torque);
    if (use_estimated_external_wrench) ext_wrench_estimator.setInitialMomentum(joint_pos, joint_vel);

    // Real-time loop
    while (1)
    {
        // Save current time point
        loop_start_time = std::chrono::steady_clock::now();
        if (!stopping_sequence_on) total_time_sec = loop_iteration_count * DT_SEC;

        // Get current state from robot sensors
        if (use_estimated_external_wrench && !stopping_sequence_on)
        {
            robot_driver.get_joint_state(joint_pos, joint_vel, joint_torque);
            ext_wrench_estimator.JntToExtWrench(joint_pos, joint_vel, torque_command, wrenches_full_model[robot_chain_full.getNrOfSegments()- 1]);
            // return_flag = controller.estimate_external_wrench(joint_pos, joint_vel, joint_torque, wrenches_full_model[robot_chain_full.getNrOfSegments()- 1]);
            if (return_flag != 0)
            {
                printf("Error in external wrench estimation\n");
                trigger_stopping_sequence = true;
            }
        }
        else robot_driver.get_robot_state(joint_pos, joint_vel, joint_torque, wrenches_full_model[robot_chain_full.getNrOfSegments()- 1]);

        // Make one control iteration (step) -> Update control commands
        return_flag = controller.step(joint_pos, joint_vel, joint_torque, wrenches_full_model[robot_chain_full.getNrOfSegments()- 1], torque_command, total_time_sec, loop_iteration_count, stop_loop_iteration_count, stopping_sequence_on);
        if (return_flag == -1) trigger_stopping_sequence = true;

        if (stopping_sequence_on) // Robot will be controlled to stop its motion and eventually lock
        {
            if (return_flag == 1) // Stop motion task completed
            {
                // Make sure that the robot is locked (freezed)
                controller.engage_lock();

                stopping_sequence_on = false;
                total_time_sec += (double)stop_loop_iteration_count / dynamics_parameter::STOPPING_MOTION_LOOP_FREQ;
                printf("Robot stopped!\n");
                printf("Control loop delay count: %d\n", control_loop_delay_count);
                controller.deinitialize();
                return 0;
            }

            // Apply joint commands using torque control interface (bypass all safety checks)
            if (controller.apply_joint_control_commands(true) == -1)
            {
                // Make sure that the robot is locked (freezed)
                controller.engage_lock();

                stopping_sequence_on = false;
                total_time_sec += (double)stop_loop_iteration_count / dynamics_parameter::STOPPING_MOTION_LOOP_FREQ;
                printf("Robot stopped!\n");
                controller.deinitialize();
                return -1;
            }

            stop_loop_iteration_count++;
            if (enforce_loop_frequency(DT_STOPPING_MICRO) != 0) control_loop_delay_count++;

            // Testing loop time
            // static double loop_time = 0.0;
            // loop_time += std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - loop_start_time).count();
            // if (stop_loop_iteration_count == 10000) 
            // {
            //     robot_driver.stop_robot_motion();
            //     printf("Stop loop time: %f\n", loop_time / 10000.0);
            //     controller.deinitialize();
            //     return 0;
            // }
        }
        else // Nominal task execution mode
        {
            // Set external wrenches for the simulation. FD solver expects wrenches to be expressed in respective link's frame... not the base frame
            if (loop_iteration_count == 3000) robot_driver.set_ext_wrenches_sim(wrenches_full_model_sim);
            if (!trigger_stopping_sequence)
            {
                // Apply joint commands using safe control interface
                if (controller.apply_joint_control_commands(false) == -1) trigger_stopping_sequence = true;
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
            // if (loop_iteration_count == 10000) 
            // {
            //     printf("Average loop time: %f\n", loop_time / 10000.0);
            //     printf("Control loop delay count: %d\n", control_loop_delay_count);
            //     trigger_stopping_sequence = true;
            // }
        }
    }

    robot_driver.stop_robot_motion();
    controller.deinitialize();
    printf("Task completed\n");
    return 0;
}

int main(int argc, char **argv)
{
    RATE_HZ              = 700; // Loop frequency in Hz
    DT_SEC               = 1.0 / static_cast<double>(RATE_HZ); // Loop period in seconds
    control_dims         = std::vector<bool>{true, true, true, // Linear
    // control_dims         = std::vector<bool>{false, false, false, // Linear
                                            //  false, false, false}; // Angular
                                             true, true, true}; // Angular
    control_dims_moveConstrained = {true, true, true, // Linear
                                    true, true, false}; // Angular

    max_command                     = (Eigen::VectorXd(NUMBER_OF_CONSTRAINTS) << 20.0, 20.0, 20.0, 120.0, 120.0, 120.0).finished();
    max_command_moveConstrained     = (Eigen::VectorXd(NUMBER_OF_CONSTRAINTS) << 20.0, 20.0, 50.0, 9.5, 9.5, 25.0).finished();
    path_parameters                 = std::vector<double>{1.5, 1.7, 0.03, 0.003, 2}; // last parameter must be min 2 (number of points)

    environment          = kinova_environment::REAL;
    robot_model_id       = kinova_model::URDF;
    id                   = robot_id::KINOVA_GEN3_1;
    desired_pose_id      = desired_pose::HOME;
    // desired_pose_id      = desired_pose::HOME_UP_2;
    // desired_pose_id      = desired_pose::HOME_UP;
    desired_control_mode = control_mode::TORQUE;
    desired_task_model   = task_model::moveTo;
    path_type            = path_types::INF_SIGN_PATH;
    motion_profile_id    = m_profile::S_CURVE;
    if (desired_pose_id == desired_pose::HOME_UP) task_time_limit_sec = 6.0;
    else task_time_limit_sec = 6.5;
    tube_speed           = 0.07;
    tube_force           = -18.5;
    tube_tolerances      = std::vector<double>{0.01, control_dims[1]? 0.02 : 0.0001, 0.02,
                                               0.09, 0.0, 0.0,
                                               tube_speed * 0.2, 0.0}; // Last tolerance is in unit of degrees - Null-space tolerance
    // Tube tolerances: x pos,    y pos,      z force, 
    //                  x torque, y torque,   null-space, 
    //                  x vel,    z_a pos/vel
    tube_tolerances_moveConstrained = std::vector<double> {0.003, 0.009, 0.003,
                                                           0.0, 0.0, 25.0,
                                                           0.0, 0.005};
    time_horizon_amplitude = 2.5;
    contact_threshold_linear  = 500.0;
    contact_threshold_angular = 500.0;
    compensate_gravity   = false;
    control_null_space   = false;
    use_mass_alternation = false;
    log_data             = true;
    use_estimated_external_wrench  = true;
    control_null_space_moveConstrained = false;

    kinova_mediator robot_driver;
    int return_flag = go_to(robot_driver, desired_pose_id);
    if (return_flag != 0) return 0;

    // Calibration function should be used only one-time. The robot must be in the zero-configuration
    // calibrate_torque_offsets(); return 0;

    // Extract robot model and if not simulation, establish connection with motor drivers
    if (!robot_driver.is_initialized()) robot_driver.initialize(robot_model_id, environment, id, 1.0 / static_cast<double>(RATE_HZ));
    if (!robot_driver.is_initialized())
    {
        printf("Robot is not initialized\n");
        return 0;
    }

    // if (robot_driver.stop_robot_motion() == -1) return 0;

    // run_test(robot_driver); return 0;

    if (run_main_control(robot_driver) == -1) return 0;
    robot_driver.deinitialize();
    return_flag = go_to(robot_driver, desired_pose_id);
    return 0;

    dynamics_controller controller(&robot_driver, RATE_HZ, compensate_gravity);

    int initial_result = define_task(&controller);
    if (initial_result != 0) return -1;

    if (desired_task_model == task_model::full_pose) 
    {
        controller.set_parameters(time_horizon_amplitude,
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
        controller.set_parameters(time_horizon_amplitude,
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
        controller.set_parameters(time_horizon_amplitude,
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
        controller.set_parameters(time_horizon_amplitude,
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
        controller.set_parameters(time_horizon_amplitude,
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
        controller.set_parameters(time_horizon_amplitude, 
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
        controller.set_parameters(time_horizon_amplitude,
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

    initial_result = controller.initialize(desired_control_mode, desired_dynamics_interface, motion_profile_id, log_data, use_estimated_external_wrench);
    if (initial_result != 0) return -1;

    controller.control();
    robot_driver.stop_robot_motion();

    controller.deinitialize();
    robot_driver.deinitialize();

    return_flag = go_to(robot_driver, desired_pose::RETRACT);
    return 0;
}