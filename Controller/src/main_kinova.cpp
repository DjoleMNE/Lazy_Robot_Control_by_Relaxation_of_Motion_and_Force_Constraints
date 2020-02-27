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
#include <safety_controller.hpp>
#include <finite_state_machine.hpp>
#include <motion_profile.hpp>

const long SECOND                    = 1000000;
const int MILLISECOND                = 1000;
const int JOINTS                     = 7;
const int NUMBER_OF_CONSTRAINTS      = 6;
int environment                      = kinova_environment::SIMULATION;
int robot_model_id                   = kinova_model::URDF;
bool compensate_gravity              = false;
double time_duration                 = 5.0f; // Duration of the example (seconds)



/*****************************
 * Example related function *
 *****************************/
int64_t GetTickUs()
{
    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);

    return (start.tv_sec * 1000000LLU) + (start.tv_nsec / 1000);
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

void run(kinova_mediator &robot_driver)
{
    int timer_count = 0;
    int64_t now = 0;
    int64_t last = 0;

    KDL::JntArray jnt_array_command(7), jnt_array_feedback(7), zero_joint_array(7);
    jnt_array_command(0) = 0.0;
    jnt_array_command(1) = 0.0;
    jnt_array_command(2) = 0.0;
    jnt_array_command(3) = 0.0;
    jnt_array_command(4) = 0.0;
    jnt_array_command(5) = 0.0;
    jnt_array_command(6) = 0.2;

    if (robot_driver.set_control_mode(control_mode::VELOCITY) == -1)
    {
        printf("Incorrect control mode\n");
        return;
    }

    // robot_driver.set_joint_positions(jnt_array_command);

    // Real-time loop
    int return_flag = 0;
    while (timer_count < (time_duration * 1000))
    {
        now = GetTickUs();

        if (now - last > 1000)
        {
            return_flag = robot_driver.set_joint_velocities(jnt_array_command);

            if (return_flag == -1)
            {
                robot_driver.stop_robot_motion();
                printf("Robot stoped: error in control\n");
                return;
            }

            robot_driver.get_joint_velocities(jnt_array_feedback);

            timer_count++;
            last = GetTickUs();
        }
    }

    robot_driver.stop_robot_motion();
    printf("Task completed\n");
}

int main(int argc, char **argv)
{
    printf("kinova MAIN Started \n");
    kinova_mediator robot_driver;
    environment        = kinova_environment::SIMULATION;
    robot_model_id     = kinova_model::URDF;
    compensate_gravity = false;
    
    // Extract robot model and if not simulation, establish connection with motor drivers
    robot_driver.initialize(robot_model_id, environment, compensate_gravity);
    if (!robot_driver.is_initialized())
    {
        printf("Robot is not initialized\n");
        return 0;
    }

    // if (robot_driver.stop_robot_motion() == -1) return 0;

    run(robot_driver);
    return 0;
}