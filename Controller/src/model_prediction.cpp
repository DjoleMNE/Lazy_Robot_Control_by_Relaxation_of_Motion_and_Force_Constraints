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

model_prediction::model_prediction(const KDL::Chain &robot_chain): 
    robot_chain_(robot_chain),
    NUMBER_OF_SEGMENTS_(robot_chain_.getNrOfSegments()),
    NUMBER_OF_FRAMES_(robot_chain_.getNrOfSegments() + 1),
    NUMBER_OF_JOINTS_(robot_chain_.getNrOfJoints()),
    fk_position_solver_(robot_chain_), 
    fk_velocity_solver_(robot_chain_)
{
    time_horizon_ = 0;
}

// Used for checking joint limits
void model_prediction::integrate_joint_space(
                                    const state_specification &current_state,
                                    state_specification &predicted_state,
                                    const double step_size_dt,
                                    const int number_of_steps, const int method)
{
    switch(method) {
        case integration_method::symplectic_euler:
            for (int i = 0; i < NUMBER_OF_JOINTS_; i++)
            {   
                //Integrate joint accelerations to velocities - Classical Euler method
                predicted_state.qd(i) = current_state.qd(i)\
                                        + current_state.qdd(i) * step_size_dt;

                //Integrate joint velocities to positions - Symplectic Euler method
                predicted_state.q(i) = current_state.q(i)\
                                        + predicted_state.qd(i) * step_size_dt;
            }
            break;

        case integration_method::predictor_corrector:
            for (int i = 0; i < NUMBER_OF_JOINTS_; i++)
            {   
                //Integrate joint accelerations to velocities - Classical Euler method
                predicted_state.qd(i) = current_state.qd(i)\
                                        + current_state.qdd(i) * step_size_dt;

                //Integrate joint velocities to joint positions - Trapezoidal method
                predicted_state.q(i) = current_state.q(i)\
                            + (predicted_state.qd(i)\
                                - current_state.qdd(i) * step_size_dt / 2.0)\
                            * step_size_dt;
            }
            break;
    }

    std::cout << "Joint Acc: " << current_state.qdd << std::endl;
    std::cout << "Joint Vel: " << predicted_state.qd << std::endl;
    std::cout << "Joint Pos: " << predicted_state.q << std::endl;

    compute_FK(predicted_state);
}

// Used for predicting future deviation from the goal state
void model_prediction::integrate_cartesian_space(
    const state_specification &current_state,
    state_specification &predicted_state,
    const double step_size,
    const int number_of_steps,
    const int method)
{
    //TODO
}

void model_prediction::compute_FK(state_specification &predicted_state)
{
    //Workaround for avoiding dynamic creation of JntArrayVel instance
    //See this class' hpp for explanation
    temp_jntarrayvel.q = predicted_state.q;
    temp_jntarrayvel.qdot = predicted_state.qd;

    //Compute angular and linear velocity of the end-effector
    fk_velocity_solver_.JntToCart(temp_jntarrayvel, temp_framevel);
    predicted_state.frame_velocity[NUMBER_OF_SEGMENTS_ - 1] = \
                                                    temp_framevel.GetTwist();

    //Compute postion and orientation of the end-effector
    fk_position_solver_.JntToCart(
                        predicted_state.q, 
                        predicted_state.frame_pose[NUMBER_OF_SEGMENTS_ - 1]);
    
    // Print Cartesian predictions
    std::cout << "End-effector Velocity: " 
        << predicted_state.frame_velocity[NUMBER_OF_SEGMENTS_ - 1]
        << std::endl;
    std::cout << "End-effector Position: " 
              << predicted_state.frame_pose[NUMBER_OF_SEGMENTS_ - 1].p
              << std::endl;
    // std::cout << "End-effector Orientation: " 
    //           << predicted_state.frame_pose[NUMBER_OF_SEGMENTS_ - 1].getRPY()
    //           << std::endl;
    std::cout << "\n" << std::endl;
}