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
    NUMBER_OF_JOINTS_(robot_chain.getNrOfJoints()),
    NUMBER_OF_SEGMENTS_(robot_chain.getNrOfSegments()),
    NUMBER_OF_FRAMES_(robot_chain.getNrOfSegments() + 1),
    NUMBER_OF_CONSTRAINTS_(dynamics_parameter::NUMBER_OF_CONSTRAINTS),
    temp_state_(NUMBER_OF_JOINTS_, NUMBER_OF_SEGMENTS_, 
                NUMBER_OF_FRAMES_, NUMBER_OF_CONSTRAINTS_),
    temp_jntarrayvel_(NUMBER_OF_JOINTS_),
    fk_position_solver_(robot_chain), 
    fk_velocity_solver_(robot_chain)
{

}

// Used for checking joint limits
void model_prediction::integrate_joint_space(
                            const state_specification &current_state,
                            std::vector<state_specification> &predicted_states,
                            const double step_size, const int number_of_steps, 
                            const int method, const bool fk_required,
                            const bool recompute_acceleration)
{
    assert(("Number of steps higher than the size of provided vector of states", 
            number_of_steps <= predicted_states.size()));  
    assert(NUMBER_OF_JOINTS_ == predicted_states[0].qd.rows()); 
    assert(NUMBER_OF_JOINTS_ == current_state.qd.rows());

    temp_state_ = current_state;

    // For each step in the future horizon
    for (int i = 0; i < number_of_steps; i++){   
        // For each robot's joint
        for (int j = 0; j < NUMBER_OF_JOINTS_; j++){                         
            integrate_to_velocity(temp_state_.qdd(j), temp_state_.qd(j),
                                  predicted_states[i].qd(j), method, step_size);

            integrate_to_position(temp_state_.qdd(j), predicted_states[i].qd(j),
                                  temp_state_.q(j), predicted_states[i].q(j), 
                                  method, step_size);
        }

        // TODO (Abstract description):
        // if(recompute_acceleration){
        //     predicted_states[i].qdd = evaluate_dynamics(predicted_states[i].q, predicted_states[i].qd)
        //     // Integrated values overwritten to be current values for the next iteration
        //     temp_state_.qdd = predicted_states[i].qdd;
        // }

        temp_state_.qd = predicted_states[i].qd;
        temp_state_.q = predicted_states[i].q;
    }

    #ifndef NDEBUG // Print joint state in Debug mode only
        std::cout << "Computed Joint Acc: " << current_state.qdd << std::endl;
        std::cout << "Integrated Joint Vel: " << predicted_states[0].qd << std::endl;
        std::cout << "Integrated Joint Pos: " << predicted_states[0].q << std::endl;
        std::cout << std::endl;
    #endif

    #ifdef NDEBUG // Print joint state in Release mode only
        std::cout << "Computed Joint Torque: " << current_state.control_torque << std::endl;
        std::cout << "Computed Joint Acc:    " << current_state.qdd << "\n" << std::endl;
        std::cout << "Current Joint Vel:     " << current_state.qd << std::endl;
        std::cout << "Integrated Joint Vel:  " << predicted_states[0].qd << std::endl;
        std::cout << "Current Joint Pos:     " << current_state.q << std::endl;
        std::cout << "\n" << std::endl;
    #endif

    if(fk_required) compute_FK(predicted_states[number_of_steps - 1]);
}

// Used for predicting future deviation from the goal state
void model_prediction::integrate_cartesian_space(
                            const state_specification &current_state,
                            std::vector<state_specification> &predicted_states,
                            const double step_size, const int number_of_steps,
                            const int method, const bool recompute_acceleration)
{
    assert(("Number of steps higher than the size of provided vector of states", 
            number_of_steps <= predicted_states.size())); 
    //TODO
}

// Scalar integration from one acceleration to one velocity
void model_prediction::integrate_to_velocity(const double &acceleration, 
                                             const double &current_velocity,
                                             double &predicted_velocity,
                                             const int method,
                                             const double dt)
{
    switch(method) {
        case integration_method::SYMPLECTIC_EULER:
            //Integrate accelerations to velocities - Classical Euler method
            predicted_velocity = current_velocity + acceleration * dt;
            break;
        
        case integration_method::PREDICTOR_CORRECTOR:
            //Integrate accelerations to velocities - Classical Euler method
            predicted_velocity = current_velocity + acceleration * dt;
            break;
        default: assert(false);
    }
}

// Scalar integration from one velocity to one position/angle
void model_prediction::integrate_to_position(const double &acceleration,
                                             const double &predicted_velocity, 
                                             const double &current_position,
                                             double &predicted_position,
                                             const int method,
                                             const double dt)
{
    switch(method) {
        case integration_method::SYMPLECTIC_EULER:
            //Integrate velocities to positions - Symplectic Euler method
            predicted_position = current_position + predicted_velocity * dt;
            break;
        
        case integration_method::PREDICTOR_CORRECTOR:
            //Integrate velocities to joint positions - Trapezoidal method
            predicted_position = current_position + dt * \
                                 (predicted_velocity - acceleration * dt / 2.0);
            break;
        default: assert(false);
    }
}

// Forward position and velocity kinematics, given the itegrated values
void model_prediction::compute_FK(state_specification &predicted_state)
{
    //Workaround for avoiding dynamic creation of JntArrayVel instance
    //See this class' hpp for explanation
    temp_jntarrayvel_.q = predicted_state.q;
    temp_jntarrayvel_.qdot = predicted_state.qd;

    //Compute angular and linear velocity of the end-effector
    fk_velocity_solver_.JntToCart(temp_jntarrayvel_, temp_framevel_);
    predicted_state.frame_velocity[NUMBER_OF_SEGMENTS_ - 1] = \
                                                    temp_framevel_.GetTwist();

    //Compute postion and orientation of the end-effector
    fk_position_solver_.JntToCart(
                        predicted_state.q, 
                        predicted_state.frame_pose[NUMBER_OF_SEGMENTS_ - 1]);
    
    // #ifndef NDEBUG // Print Cartesian state in Debug mode only
    // std::cout << "End-effector Velocity: " 
    //     << predicted_state.frame_velocity[NUMBER_OF_SEGMENTS_ - 1]
    //     << std::endl;
    // std::cout << "End-effector Position: " 
    //           << predicted_state.frame_pose[NUMBER_OF_SEGMENTS_ - 1].p
    //           << std::endl;
    // std::cout << "End-effector Orientation: " 
    //           << predicted_state.frame_pose[NUMBER_OF_SEGMENTS_ - 1].getRPY()
    //           << std::endl;
    // std::cout << "\n" << std::endl;
    // #endif
}