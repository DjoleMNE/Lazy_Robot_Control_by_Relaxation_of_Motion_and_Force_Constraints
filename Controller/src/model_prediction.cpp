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
    fk_vereshchagin_(robot_chain), fk_solver_result_(0)
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
        std::cout << "\nIntegrated Joint Vel: " << predicted_states[0].qd << std::endl;
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

    if(fk_required) compute_FK(predicted_states[0]);
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
    // Compute angular and linear velocity of the end-effector
    // Compute postion and orientation of the end-effector
    fk_solver_result_ = fk_vereshchagin_.JntToCart(predicted_state.q, 
                                                   predicted_state.qd, 
                                                   predicted_state.frame_pose, 
                                                   predicted_state.frame_velocity);
    if(fk_solver_result_ != 0) 
        std::cout << "Warning: FK solver returned an error! " << fk_solver_result_ << std::endl;
    
    #ifndef NDEBUG // Print Cartesian state in Debug mode only
        std::cout << "Predicted End-effector Position: " 
                << predicted_state.frame_pose[NUMBER_OF_SEGMENTS_ - 1].p
                << std::endl;
        // std::cout << "Predicted End-effector Orientation: " 
        //         << predicted_state.frame_pose[NUMBER_OF_SEGMENTS_ - 1].getRPY()
        //         << "\n" << std::endl;
        std::cout << "Predicted End-effector Velocity: " 
            << predicted_state.frame_velocity[NUMBER_OF_SEGMENTS_ - 1]
            << std::endl;
    #endif
}

/*
    Used for predicting future deviation from the goal state
    Saves the intermediate states computed throughout the itegration
*/
void model_prediction::integrate_cartesian_space(
                            const state_specification &current_state,
                            std::vector<state_specification> &predicted_states,
                            const double dt, const int number_of_steps)
{
    assert(("Number of steps higher than the size of provided vector of states", 
            number_of_steps <= predicted_states.size())); 
    //TODO
}

/*
    Used for predicting future deviation from the goal state.
    The intermediate states computed throughout the itegration are not saved.
    The function expects constant SCREW twist (not body-fixed twist).
    I.e. both the reference frame and the reference point are in the base link.
*/
void model_prediction::integrate_cartesian_space(
                            const state_specification &current_state,
                            state_specification &predicted_state,
                            const double dt, const int number_of_steps)
{
    // Only frame/s and velocities of one segment should be passed! change func def!
    assert(NUMBER_OF_SEGMENTS_ == current_state.frame_velocity.size());
    assert(NUMBER_OF_SEGMENTS_ == predicted_state.frame_velocity.size()); 

    std::cout << "Measured End-effector Position:       " 
              << current_state.frame_pose[NUMBER_OF_SEGMENTS_ - 1].p  << std::endl;

    std::cout << "Measured End-effector Orientation:       \n" 
            << current_state.frame_pose[NUMBER_OF_SEGMENTS_ - 1].M  << std::endl;

    predicted_state.frame_pose[NUMBER_OF_SEGMENTS_ - 1] = \
        KDL::addDelta(current_state.frame_pose[NUMBER_OF_SEGMENTS_ - 1], 
                      current_state.frame_velocity[NUMBER_OF_SEGMENTS_ - 1], 
                      dt);
    std::cout << "Integrated End-effector Position 1:   " 
              << predicted_state.frame_pose[NUMBER_OF_SEGMENTS_ - 1].p  << std::endl;
    
    std::cout << "Integrated End-effector Orientation 1:       \n" 
            << predicted_state.frame_pose[NUMBER_OF_SEGMENTS_ - 1].M  << std::endl;
    
    // predicted_state = current_state;
    
    // predicted_state.frame_pose[NUMBER_OF_SEGMENTS_ - 1].Integrate(current_state.frame_velocity[NUMBER_OF_SEGMENTS_ - 1], 1.0 / dt);
    // std::cout << "Integrated End-effector Position 2:   " 
    //           << predicted_state.frame_pose[NUMBER_OF_SEGMENTS_ - 1].p  << std::endl;
    // std::cout << "Integrated End-effector Orientation 2:       \n" 
    //           << predicted_state.frame_pose[NUMBER_OF_SEGMENTS_ - 1].M  << std::endl;
    
    normalize_rot_matrix(predicted_state.frame_pose[NUMBER_OF_SEGMENTS_ - 1].M);

    #ifndef NDEBUG
        measured_data_file_.open(MEASURED_DATA_PATH_);
        save_pose_to_file(measured_data_file_, 
                        current_state.frame_pose[NUMBER_OF_SEGMENTS_ - 1]);

        predicted_data_file_.open(PREDICTED_DATA_PATH_);
        save_pose_to_file(predicted_data_file_, 
                        predicted_state.frame_pose[NUMBER_OF_SEGMENTS_ - 1]);
    #endif
    
    // This should go in hpp file or in dynamics controller
    KDL::Vector position_error(0.0, 0.0, 0.0);
    KDL::Vector orientation_error(0.0, 0.0, 0.0);
    KDL::Frame error_frame = KDL::Frame::Identity();

    // Relative motion necessary to go from predicted to desired/measured
    // Frame of desired/measured w.r.t predicted frame
    error_frame = predicted_state.frame_pose[NUMBER_OF_SEGMENTS_ - 1].Inverse() * current_state.frame_pose[NUMBER_OF_SEGMENTS_ - 1];
    position_error = error_frame.p;
    // Check docs for RPY due to issue with non uniqueness of values
    error_frame.M.GetRPY(orientation_error(0), orientation_error(1), orientation_error(2));

    std::cout << "Error Position:                        " << position_error  << std::endl;
    std::cout << "Error Orientation:                     " << orientation_error  << std::endl;
}

void model_prediction::save_pose_to_file(std::ofstream &pose_data_file, 
                                         const KDL::Frame &frame_pose)
{
    if (!pose_data_file.is_open()) {
        std::cout << "Unable to open the file"<< std::endl;
    }
    
    for(int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++)
            pose_data_file << frame_pose.M(i, j) << std::endl;
    }

    for (int j = 0; j < 3; j++)
        pose_data_file << frame_pose.p(j)<< std::endl;

    pose_data_file.close();
}

void model_prediction::normalize_rot_matrix(KDL::Rotation &rot_martrix)
{
    // This should go in hpp file
    double x, y, z, w;

    rot_martrix.GetQuaternion(x, y, z, w);
    rot_martrix = KDL::Rotation::Quaternion(x, y, z, w);
}