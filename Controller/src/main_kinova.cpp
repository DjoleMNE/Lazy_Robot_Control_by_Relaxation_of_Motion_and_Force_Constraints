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

float time_duration = 5.0f; // Duration of the example (seconds)

/*****************************
 * Example related function *
 *****************************/
int64_t GetTickUs()
{
    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);

    return (start.tv_sec * 1000000LLU) + (start.tv_nsec / 1000);
}

void run(kinova_mediator &robot_driver)
{
    int timer_count = 0;
    int64_t now = 0;
    int64_t last = 0;

    KDL::JntArray jnt_array(7);
    jnt_array(0) = 0.0;
    jnt_array(1) = 0.0;
    jnt_array(2) = 0.0;
    jnt_array(3) = 0.0;
    jnt_array(4) = 0.0;
    jnt_array(5) = 0.0;
    jnt_array(6) = 0.1;

    // Real-time loop
    while (timer_count < (time_duration * 1000))
    {
        now = GetTickUs();

        if (now - last > 1000)
        {
            if (robot_driver.set_joint_velocities(jnt_array) == -1)
            {
                robot_driver.stop_robot_motion();
                return;
            }
            timer_count++;
            last = GetTickUs();
        }
    }
    robot_driver.stop_robot_motion();
    printf("task completed\n");
}

int main(int argc, char **argv)
{
    printf("kinova MAIN Started \n");
    kinova_mediator robot_driver;
    int environment          = kinova_environment::SIMULATION;
    int robot_model_id       = kinova_model::URDF;
    bool compensate_gravity  = false;

    // Extract robot model and if not simulation, establish connection with motor drivers
    robot_driver.initialize(robot_model_id, environment, compensate_gravity);
    if (!robot_driver.is_initialized())
    {
        printf("Robot is not initialized\n");
        return 0;
    }

    run(robot_driver);
    // robot_driver.deinitialize();
    return 0;
}