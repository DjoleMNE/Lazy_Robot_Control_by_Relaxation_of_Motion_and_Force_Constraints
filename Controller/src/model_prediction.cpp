/*
Author(s): Djordje Vukcevic, Sven Schneider
Institute: Hochschule Bonn-Rhein-Sieg

Copyright (c) [2019]

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
    NUM_OF_JOINTS_(robot_chain.getNrOfJoints()),
    NUM_OF_SEGMENTS_(robot_chain.getNrOfSegments()),
    NUM_OF_FRAMES_(robot_chain.getNrOfSegments() + 1),
    NUM_OF_CONSTRAINTS_(dynamics_parameter::NUMBER_OF_CONSTRAINTS),
    END_EFF_(NUM_OF_SEGMENTS_ - 1),
    fk_vereshchagin_(robot_chain),
    temp_state_(NUM_OF_JOINTS_, NUM_OF_SEGMENTS_, NUM_OF_FRAMES_, NUM_OF_CONSTRAINTS_),
    temp_pose_(KDL::Frame::Identity())
{

}

// Used for checking joint limits
void model_prediction::integrate_joint_space(
                            const state_specification &current_state,
                            std::vector<state_specification> &predicted_states,
                            const double dt_sec, const int num_of_steps, 
                            const int method, const bool fk_required,
                            const bool recompute_acceleration)
{
    assert(("Number of steps higher than the size of provided vector of states", 
            num_of_steps <= predicted_states.size()));  
    assert(NUM_OF_JOINTS_ == predicted_states[0].qd.rows()); 
    assert(NUM_OF_JOINTS_ == current_state.qd.rows());

    temp_state_ = current_state;

    // For each step in the future horizon
    for (int i = 0; i < num_of_steps; i++){   

        integrate_to_velocity(temp_state_.qdd, temp_state_.qd,
                              predicted_states[i].qd, method, dt_sec);

        integrate_to_position(temp_state_.qdd, predicted_states[i].qd,
                              temp_state_.q, predicted_states[i].q, 
                              method, dt_sec);

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
        // std::cout << "\nIntegrated Joint Vel: " << predicted_states[0].qd << std::endl;
        // std::cout << "Integrated Joint Pos: " << predicted_states[0].q << std::endl;
    #endif

    #ifdef NDEBUG // Print joint state in Release mode only
        // std::cout << "Computed Joint Torque: " << current_state.control_torque << std::endl;
        // std::cout << "Computed Joint Acc:    " << current_state.qdd << "\n" << std::endl;
        // std::cout << "Current Joint Vel:     " << current_state.qd << std::endl;
        // std::cout << "Integrated Joint Vel:  " << predicted_states[0].qd << std::endl;
        // std::cout << "Current Joint Pos:     " << current_state.q << std::endl;
        // std::cout << "Integrated Joint Pos:  " << predicted_states[0].q << std::endl;
        printf("\n");
    #endif

    if(fk_required) compute_FK(predicted_states[0]);
}

// Vector integration from joint acceleration to joint velocity
void model_prediction::integrate_to_velocity(const KDL::JntArray &acceleration, 
                                             const KDL::JntArray &current_velocity,
                                             KDL::JntArray &predicted_velocity,
                                             const int method,
                                             const double dt_sec)
{
    switch(method) {
        case integration_method::SYMPLECTIC_EULER:
            //Integrate accelerations to velocities - Classical Euler method
            predicted_velocity.data = current_velocity.data + acceleration.data * dt_sec;
            break;
        
        case integration_method::PREDICTOR_CORRECTOR:
            //Integrate accelerations to velocities - Classical Euler method
            predicted_velocity.data = current_velocity.data + acceleration.data * dt_sec;
            break;
        default: assert(false);
    }
}

// Vector integration from joint velocity to joint position/angle
void model_prediction::integrate_to_position(const KDL::JntArray &acceleration,
                                             const KDL::JntArray &predicted_velocity, 
                                             const KDL::JntArray &current_position,
                                             KDL::JntArray &predicted_position,
                                             const int method,
                                             const double dt_sec)
{
    switch(method) {
        case integration_method::SYMPLECTIC_EULER:
            //Integrate velocities to positions - Symplectic Euler method
            predicted_position.data = current_position.data + predicted_velocity.data * dt_sec;
            break;
        
        case integration_method::PREDICTOR_CORRECTOR:
            //Integrate velocities to joint positions - Trapezoidal method
            predicted_position.data = current_position.data + dt_sec * \
                                 (predicted_velocity.data - acceleration.data * dt_sec / 2.0);
            break;
        default: assert(false);
    }
}

/*
    Used for predicting future deviation from the goal state.
    The intermediate states computed throughout the integration are NOT saved.
    The function expects constant Pose twist (not screw twist).
*/
void model_prediction::integrate_cartesian_space(
                            const state_specification &current_state,
                            state_specification &predicted_state,
                            const double dt_sec, const int num_of_steps)
{
    // assert(dt_sec > 0.0);
    assert(num_of_steps >= 1);
    assert(NUM_OF_SEGMENTS_ == current_state.frame_velocity.size());
    assert(NUM_OF_SEGMENTS_ == predicted_state.frame_velocity.size()); 

    temp_pose_ = current_state.frame_pose[END_EFF_];
    KDL::Twist pose_twist = current_state.frame_velocity[END_EFF_] * dt_sec;
    KDL::Twist body_fixed_twist; 

// Save constant data to a file for visualization purposes.
#ifndef NDEBUG
        twist_data_file_.open(prediction_parameter::TWIST_DATA_PATH);
        save_twist_to_file(twist_data_file_, pose_twist);
        twist_data_file_.close();

        current_pose_data_file_.open(prediction_parameter::CURRENT_POSE_DATA_PATH);
        save_pose_to_file(current_pose_data_file_, current_state.frame_pose[END_EFF_]);
        current_pose_data_file_.close();
        
        predicted_pose_data_file_.open(prediction_parameter::PREDICTED_POSE_DATA_PATH);
#endif

    geometry::orthonormalize_rot_matrix(temp_pose_.M);
    assert(("Current rotation matrix", geometry::is_rotation_matrix(temp_pose_.M)));
    
#ifdef NDEBUG
    if(!geometry::is_rotation_matrix(temp_pose_.M)) printf("Current Matrix is not rotation!");
#endif 

    for (int i = 0; i < num_of_steps; i++)
    {
        body_fixed_twist = temp_pose_.M.Inverse(pose_twist);
        temp_pose_ = integrate_pose(temp_pose_, body_fixed_twist, true, true);
        geometry::orthonormalize_rot_matrix(temp_pose_.M);
        assert(("Integrated rotation matrix", geometry::is_rotation_matrix(temp_pose_.M)));

#ifdef NDEBUG
        if(!geometry::is_rotation_matrix(temp_pose_.M)) printf("Integrated Matrix is not rotation!");
#endif

#ifndef NDEBUG
        save_pose_to_file(predicted_pose_data_file_, temp_pose_);
#endif
    }
    
#ifndef NDEBUG
        // std::cout << "Measured End-effector Position:\n" 
        //           << current_state.frame_pose[END_EFF_].p 
        //           << std::endl;

        // std::cout << "Measured End-effector Orientation:\n" 
        //           << current_state.frame_pose[END_EFF_].M 
        //           << std::endl;

        // std::cout << "Integrated End-effector Position:\n" 
        //           << temp_pose_.p  << std::endl;
        
        // std::cout << "Integrated End-effector Orientation:\n" 
        //           << temp_pose_.M  << std::endl;

        predicted_pose_data_file_.close();
#endif
    
    predicted_state.frame_pose[END_EFF_] = temp_pose_;
}

/**
 * Calculates Exponential map for both translation and rotation
 * Input: 
 * 		- current pose to be integrated
 * 		- current "body-fixed" twist scaled with delta time
 * 		- flag for rescaling rotation if it's out of 0 - PI range
 * 		- flag for decoupling integration of linear and angular parts  
 * Output: tranformation (KDL::Frame) of the integrated (predicted) pose
 * Code is based on formulas given in books:
 * "Modern Robotics", 2017, F.C. Park
 * "Robot Kinematics and Dynamics", 2010, Herman Bruyninckx.
*/
KDL::Frame model_prediction::integrate_pose(const KDL::Frame &current_pose,
                                            KDL::Twist &current_twist, 
                                            const bool rescale_rotation,
								            const bool decouple_dimensions) 
{
    if(rescale_rotation) geometry::rescale_angular_twist(current_twist.rot);

    if(decouple_dimensions)
    {   
        /** 
         * Exponential map on SO(3) only: Decoupled calculation 
         * See "Modern Robotics" Book, 2017, sections 9.2.1 and 11.3.3.
        */
        return current_pose * KDL::Frame(geometry::exp_map_so3(current_twist.rot), 
                                         current_twist.vel);
    }
    
    return current_pose * geometry::exp_map_se3(current_twist);
}

void model_prediction::save_pose_to_file(std::ofstream &pose_data_file, 
                                         const KDL::Frame &pose)
{
    if (!pose_data_file.is_open()) 
    {
        std::cout << "Unable to open the pose file" << std::endl;
    }
    
    for(int i = 0; i < 9; i++) pose_data_file << pose.M.data[i] << std::endl;
    for(int j = 0; j < 3; j++) pose_data_file << pose.p(j) << std::endl;
}

void model_prediction::save_twist_to_file(std::ofstream &twist_data_file, 
                                         const KDL::Twist &twist)
{
    if (!twist_data_file.is_open())
    {
        std::cout << "Unable to open the twist file" << std::endl;
    }
    for (int j = 0; j < 6; j++) twist_data_file << twist(j) << std::endl;
}

// Forward position and velocity kinematics, given the itegrated values
void model_prediction::compute_FK(state_specification &predicted_state)
{
    int fk_solver_result_;
    // Compute angular and linear velocity of the end-effector
    // Compute position and orientation of the end-effector
    fk_solver_result_ = fk_vereshchagin_.JntToCart(predicted_state.q, 
                                                   predicted_state.qd, 
                                                   predicted_state.frame_pose, 
                                                   predicted_state.frame_velocity);
    if(fk_solver_result_ != 0) 
    {
        std::cout << "Warning: FK solver returned an error! " << fk_solver_result_ << std::endl;
    }
    
    #ifndef NDEBUG // Print Cartesian state in Debug mode only
        std::cout << "Predicted End-effector Position: \n" 
                  << predicted_state.frame_pose[END_EFF_].p
                  << std::endl;
        std::cout << "Predicted End-effector Orientation: \n" 
                  << predicted_state.frame_pose[END_EFF_].M
                  << "\n" << std::endl;
        std::cout << "Predicted End-effector Velocity: \n" 
                  << predicted_state.frame_velocity[END_EFF_]
                  << std::endl;
    #endif
}