/*
Author(s): Djordje Vukcevic, Sven Schneider
Institute: Hochschule Bonn-Rhein-Sieg

Copyright (c) [2018]

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
#include <model_prediction.hpp>

model_prediction::model_prediction(KDL::Chain &arm_chain): 
    arm_chain_(arm_chain),
    NUMBER_OF_SEGMENTS_(arm_chain.getNrOfSegments()),
    NUMBER_OF_JOINTS_(arm_chain.getNrOfJoints()),
    fk_position_solver_(arm_chain), 
    fk_velocity_solver_(arm_chain)
{
    time_horizon_ = 0;
}

//Make predictions via integration method
void model_prediction::integrate(const state_specification &current_state,
                                 state_specification &predicted_state,
                                 const double step_size_dt,
                                 const int number_of_steps)
{
    // How long the future is: delta time * number of steps in future
    time_horizon_ = step_size_dt * number_of_steps; 
    double joint_velocity_limits[5] = {1.5707, 0.8, 1.0, 1.5707, 1.5707};

    //Euler method
    for (int i = 0; i < NUMBER_OF_JOINTS_; i++)
    {   
        //Integrate from joint accelerations to joint velocities
        predicted_state.qd(i) = current_state.qd(i)\
                                        + current_state.qdd(i) * time_horizon_;

        //If joint limit reached, stop the program
        if (abs(predicted_state.qd(i)) > joint_velocity_limits[i])
        {
            std::cout << "Limit reached on: " 
                        << predicted_state.qd(i)
                        << " " << i << std::endl;
        }
        assert(abs(predicted_state.qd(i)) <= joint_velocity_limits[i]);

        //Integrate from joint velocities to joint positions
        predicted_state.q(i) = current_state.q(i)\
                            + (predicted_state.qd(i)\
                                - current_state.qdd(i) * time_horizon_ / 2.0)\
                            * time_horizon_;
    }
    //Workaround for avoiding run-time creation of JntArrayVel instance
    //See this hpp for explanation
    temp_jntarrayvel.q = predicted_state.q;
    temp_jntarrayvel.qdot = predicted_state.qd;

    //Compute angular and linear velocity of the end-effector
    fk_velocity_solver_.JntToCart(
                    temp_jntarrayvel, 
                    predicted_state.frame_velocity[NUMBER_OF_SEGMENTS_ - 1]);

    //Compute postion and orientation of the end-effector
    fk_position_solver_.JntToCart(
            predicted_state.q, 
            predicted_state.frame_pose[NUMBER_OF_SEGMENTS_ - 1]);

    // current_state = predicted_state;
    
    // Print Cartesian predictions
    // std::cout << "End-effector Velocity: " 
    //     << predicted_state.frame_velocity[NUMBER_OF_SEGMENTS_ - 1].GetTwist()
    //     << std::endl;
    // std::cout << "End-effector Pose: " 
    //           << predicted_state.frame_pose[NUMBER_OF_SEGMENTS_ - 1].p
    //           << std::endl;
}