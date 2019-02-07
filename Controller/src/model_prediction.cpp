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
#define EPSILON 0.01  // margin to allow for rounding errors
#define MIN_ANGLE 1e-6

model_prediction::model_prediction(const KDL::Chain &robot_chain): 
    NUMBER_OF_JOINTS_(robot_chain.getNrOfJoints()),
    NUMBER_OF_SEGMENTS_(robot_chain.getNrOfSegments()),
    NUMBER_OF_FRAMES_(robot_chain.getNrOfSegments() + 1),
    NUMBER_OF_CONSTRAINTS_(dynamics_parameter::NUMBER_OF_CONSTRAINTS),
    fk_solver_result_(0), fk_vereshchagin_(robot_chain), 
    q_x_(0.0), q_y_(0.0), q_z_(0.0), q_w_(0.0), // Temp quaternion parameters
    rot_norm_(0.0), temp_state_(NUMBER_OF_JOINTS_, NUMBER_OF_SEGMENTS_, 
                                NUMBER_OF_FRAMES_, NUMBER_OF_CONSTRAINTS_),
    body_fixed_twist_(KDL::Twist::Zero()), normalized_twist_(KDL::Twist::Zero()), 
    temp_pose_(KDL::Frame::Identity()),
    skew_rotation_sin_(KDL::Rotation::Identity()), 
    skew_rotation_cos_(KDL::Rotation::Identity()),
    CURRENT_POSE_DATA_PATH_(prediction_parameter::CURRENT_POSE_DATA_PATH),
    PREDICTED_POSE_DATA_PATH_(prediction_parameter::PREDICTED_POSE_DATA_PATH),
    TWIST_DATA_PATH_(prediction_parameter::TWIST_DATA_PATH)
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
    assert(NUMBER_OF_JOINTS_ == predicted_states[0].qd.rows()); 
    assert(NUMBER_OF_JOINTS_ == current_state.qd.rows());

    temp_state_ = current_state;

    // For each step in the future horizon
    for (int i = 0; i < num_of_steps; i++){   
        // For each robot's joint
        for (int j = 0; j < NUMBER_OF_JOINTS_; j++){
            integrate_to_velocity(temp_state_.qdd(j), temp_state_.qd(j),
                                  predicted_states[i].qd(j), method, dt_sec);

            integrate_to_position(temp_state_.qdd(j), predicted_states[i].qd(j),
                                  temp_state_.q(j), predicted_states[i].q(j), 
                                  method, dt_sec);
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
                                             const double dt_sec)
{
    switch(method) {
        case integration_method::SYMPLECTIC_EULER:
            //Integrate accelerations to velocities - Classical Euler method
            predicted_velocity = current_velocity + acceleration * dt_sec;
            break;
        
        case integration_method::PREDICTOR_CORRECTOR:
            //Integrate accelerations to velocities - Classical Euler method
            predicted_velocity = current_velocity + acceleration * dt_sec;
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
                                             const double dt_sec)
{
    switch(method) {
        case integration_method::SYMPLECTIC_EULER:
            //Integrate velocities to positions - Symplectic Euler method
            predicted_position = current_position + predicted_velocity * dt_sec;
            break;
        
        case integration_method::PREDICTOR_CORRECTOR:
            //Integrate velocities to joint positions - Trapezoidal method
            predicted_position = current_position + dt_sec * \
                                 (predicted_velocity - acceleration * dt_sec / 2.0);
            break;
        default: assert(false);
    }
}

/*
    Used for predicting future deviation from the goal state.
    The intermediate states computed throughout the itegration are NOT saved.
    The function expects constant Pose twist (not screw twist).
*/
void model_prediction::integrate_cartesian_space(
                            const state_specification &current_state,
                            state_specification &predicted_state,
                            const double dt_sec, const int num_of_steps)
{
    assert(dt_sec > 0.0 && dt_sec <= 1.0);
    assert(num_of_steps >= 1);
    assert(NUMBER_OF_SEGMENTS_ == current_state.frame_velocity.size());
    assert(NUMBER_OF_SEGMENTS_ == predicted_state.frame_velocity.size()); 

    temp_pose_ = current_state.frame_pose[NUMBER_OF_SEGMENTS_ - 1];
     
    body_fixed_twist_(0) = 0.5;
    body_fixed_twist_(1) = 0.0;
    body_fixed_twist_(2) = 0.6;

    body_fixed_twist_(3) = 0.001;
    body_fixed_twist_(4) = 0.0;
    body_fixed_twist_(5) = 0.0;

    body_fixed_twist_ = temp_pose_.M.Inverse(body_fixed_twist_) * dt_sec;

    // body_fixed_twist_ = \
    //     temp_pose_.M.Inverse(current_state.frame_velocity[NUMBER_OF_SEGMENTS_ - 1]) * dt_sec;

    /** 
     * Justification for the following assertion found in: 
     * "Practical Parameterization of Rotations Using the Exponential Map",
     *  by F. Sebastian Grassia, page 7.
     * If the following assert fails, delta time must be reduced and 
     * if necessary number of steps needs to be increased. Basically the angle
     * step is to high for exp map.
     * Alternative is to "rephrase" the orientation step, before applying it via 
     * the exponential map and avoid singularity which occurs in mapping from 
     * R^3 to SO(3) if angle step is too high. 
     * See the paper from above for more details. 
    */
    assert((body_fixed_twist_.rot.Norm() <= M_PI));
    assert(("Current rotation matrix", is_rotation_matrix(temp_pose_.M)));
    
    for (int i = 0; i < num_of_steps; i++){
        // temp_pose_ = KDL::addDelta(temp_pose_, body_fixed_twist_);
        temp_pose_ = integrate_pose(temp_pose_, body_fixed_twist_);
        // temp_pose_.Integrate(body_fixed_twist_, 1.0);
        normalize_rot_matrix(temp_pose_.M);
        assert(("Integrated rotation matrix", is_rotation_matrix(temp_pose_.M)));
    }
    
    #ifndef NDEBUG
        std::cout << "Measured End-effector Position:\n" 
                  << current_state.frame_pose[NUMBER_OF_SEGMENTS_ - 1].p 
                  << std::endl;

        std::cout << "Measured End-effector Orientation:\n" 
                  << current_state.frame_pose[NUMBER_OF_SEGMENTS_ - 1].M 
                  << std::endl;

        std::cout << "Integrated End-effector Position 1:\n" 
                  << temp_pose_.p  << std::endl;
        
        std::cout << "Integrated End-effector Orientation 1:\n" 
                  << temp_pose_.M  << std::endl;

        std::cout << "Delta Angle: " << body_fixed_twist_.rot.Norm() << std::endl; 
        twist_data_file_.open(TWIST_DATA_PATH_);
        save_twist_to_file(twist_data_file_, body_fixed_twist_);

        current_pose_data_file_.open(CURRENT_POSE_DATA_PATH_);
        save_pose_to_file(current_pose_data_file_, 
                          current_state.frame_pose[NUMBER_OF_SEGMENTS_ - 1]);

        predicted_pose_data_file_.open(PREDICTED_POSE_DATA_PATH_);
        save_pose_to_file(predicted_pose_data_file_, temp_pose_);
    #endif
    
    predicted_state.frame_pose[NUMBER_OF_SEGMENTS_ - 1] = temp_pose_;
}

/**
 * Code is based on formulas given in books:
 * "Modern Robotics", F.C.Park
 * "Robot Kinematics and Dynamics", Herman B.
*/
KDL::Frame model_prediction::integrate_pose(const KDL::Frame &current_pose,
                                            const KDL::Twist &current_twist) 
{
    rot_norm_ = current_twist.rot.Norm();
    if (rot_norm_ < MIN_ANGLE) {
        return current_pose * KDL::Frame(KDL::Rotation::Identity(),
                                         current_twist.vel);
    } else {
        normalized_twist_.vel = current_twist.vel / rot_norm_;
        normalized_twist_.rot = current_twist.rot / rot_norm_;
        return current_pose * KDL::Frame(angular_exp_map(normalized_twist_, rot_norm_),
                                         linear_exp_map(normalized_twist_, rot_norm_));
    }
}

// Calculate  exponential map for angular part of the given screw twist
// Given twist vector must be normalized!
KDL::Rotation model_prediction::angular_exp_map(const KDL::Twist &normalized_twist_,
                                                const double &rot_norm)
{   
    return KDL::Rotation::Rot2(normalized_twist_.rot, rot_norm);
}

// Calculate exponential map for linear part of the given screw twist
// Given twist vector must be normalized!
KDL::Vector model_prediction::linear_exp_map(const KDL::Twist &normalized_twist_,
                                             const double &rot_norm)
{
    // Normalize and Convert rotation vector to a skew matrix 
    skew_rotation_cos_ = skew_matrix( normalized_twist_.rot );
    skew_rotation_sin_ = skew_rotation_cos_ * skew_rotation_cos_;

    return (matrix_addition(KDL::Rotation(rot_norm,      0.0,      0.0,
                                               0.0, rot_norm,      0.0,
                                               0.0,      0.0, rot_norm),
                            matrix_addition(scale_matrix( skew_rotation_cos_, 1        - cos(rot_norm) ),
                                            scale_matrix( skew_rotation_sin_, rot_norm - sin(rot_norm) )
                                           )
                           )
           ) * normalized_twist_.vel;
}

//Converts a 3D vector to an skew matrix representation
KDL::Rotation model_prediction::skew_matrix(const KDL::Vector &vector)
{
    return KDL::Rotation( 0.0,       -vector(2),  vector(1),
                          vector(2),        0.0, -vector(0),
                         -vector(1),  vector(0),        0.0);
}

//Scale a 3x3 matrix with a scalar number
KDL::Rotation model_prediction::scale_matrix(const KDL::Rotation &matrix,
                                             const double &scale)
{
    return KDL::Rotation(matrix.data[0] * scale, matrix.data[1] * scale, matrix.data[2] * scale,
                         matrix.data[3] * scale, matrix.data[4] * scale, matrix.data[5] * scale,
                         matrix.data[6] * scale, matrix.data[7] * scale, matrix.data[8] * scale);
}

KDL::Rotation model_prediction::matrix_addition(const KDL::Rotation &matrix_1,
                                                const KDL::Rotation &matrix_2)
{
    return KDL::Rotation(matrix_1.data[0] + matrix_2.data[0], matrix_1.data[1] + matrix_2.data[1], matrix_1.data[2] + matrix_2.data[2],
                         matrix_1.data[3] + matrix_2.data[3], matrix_1.data[4] + matrix_2.data[4], matrix_1.data[5] + matrix_2.data[5],
                         matrix_1.data[6] + matrix_2.data[6], matrix_1.data[7] + matrix_2.data[7], matrix_1.data[8] + matrix_2.data[8]);
}


void model_prediction::save_pose_to_file(std::ofstream &pose_data_file, 
                                         const KDL::Frame &pose)
{
    if (!pose_data_file.is_open()) {
        std::cout << "Unable to open the pose file"<< std::endl;
    }
    
    for(int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++)
            pose_data_file << pose.M(i, j) << std::endl;
    }

    for (int j = 0; j < 3; j++)
        pose_data_file << pose.p(j)<< std::endl;

    pose_data_file.close();
}

void model_prediction::save_twist_to_file(std::ofstream &twist_data_file, 
                                         const KDL::Twist &twist)
{
    if (!twist_data_file.is_open()) {
        std::cout << "Unable to open the twist file"<< std::endl;
    }
    for (int j = 0; j < 6; j++) twist_data_file << twist(j)<< std::endl;
    twist_data_file.close();
}

void model_prediction::normalize_rot_matrix(KDL::Rotation &rot_martrix)
{
    // Internally KDL normalizes the quaternion before return
    // Note that it is still not proven to be right way to do it!
    rot_martrix.GetQuaternion(q_x_, q_y_, q_z_, q_w_);
    rot_martrix = KDL::Rotation::Quaternion(q_x_, q_y_, q_z_, q_w_);
}

// Forward position and velocity kinematics, given the itegrated values
void model_prediction::compute_FK(state_specification &predicted_state)
{
    // Compute angular and linear velocity of the end-effector
    // Compute position and orientation of the end-effector
    fk_solver_result_ = fk_vereshchagin_.JntToCart(predicted_state.q, 
                                                   predicted_state.qd, 
                                                   predicted_state.frame_pose, 
                                                   predicted_state.frame_velocity);
    if(fk_solver_result_ != 0) 
        std::cout << "Warning: FK solver returned an error! " << fk_solver_result_ << std::endl;
    
    #ifndef NDEBUG // Print Cartesian state in Debug mode only
        std::cout << "Predicted End-effector Position: \n" 
                  << predicted_state.frame_pose[NUMBER_OF_SEGMENTS_ - 1].p
                  << std::endl;
        std::cout << "Predicted End-effector Orientation: \n" 
                  << predicted_state.frame_pose[NUMBER_OF_SEGMENTS_ - 1].M
                  << "\n" << std::endl;
        std::cout << "Predicted End-effector Velocity: \n" 
                  << predicted_state.frame_velocity[NUMBER_OF_SEGMENTS_ - 1]
                  << std::endl;
    #endif
}

/**
* Method checks that the input is a pure rotation matrix 'm'.
* The condition for this is:
* R' * R = I
* and
* det(R) = 1
* Source: http://www.euclideanspace.com/maths/algebra/matrix/orthogonal/rotation/
*/
bool model_prediction::is_rotation_matrix(const KDL::Rotation &m)
{
	if (abs(m(0, 0)*m(0, 1) + m(0, 1)*m(1, 1) + m(0, 2)*m(1, 2))     > EPSILON) return false;
	if (abs(m(0, 0)*m(2, 0) + m(0, 1)*m(2, 1) + m(0, 2)*m(2, 2))     > EPSILON) return false;
	if (abs(m(1, 0)*m(2, 0) + m(1, 1)*m(2, 1) + m(1, 2)*m(2, 2))     > EPSILON) return false;
	if (abs(m(0, 0)*m(0, 0) + m(0, 1)*m(0, 1) + m(0, 2)*m(0, 2) - 1) > EPSILON) return false;
	if (abs(m(1, 0)*m(1, 0) + m(1, 1)*m(1, 1) + m(1, 2)*m(1, 2) - 1) > EPSILON) return false;
	if (abs(m(2, 0)*m(2, 0) + m(2, 1)*m(2, 1) + m(2, 2)*m(2, 2) - 1) > EPSILON) return false;
	return (abs(determinant(m) - 1) < EPSILON);
}

/**
 * code is defined here:
 * https://www.euclideanspace.com/maths/algebra/matrix/functions/determinant/threeD/
*/
double model_prediction::determinant(const KDL::Rotation &m) 
{
    return m(0, 0) * m(1, 1) * m(2, 2) + \
           m(0, 1) * m(1, 2) * m(2, 0) + \
           m(0, 2) * m(1, 0) * m(2, 1) - \
           m(0, 0) * m(1, 2) * m(2, 1) - \
           m(0, 1) * m(1, 0) * m(2, 2) - \
           m(0, 2) * m(1, 1) * m(2, 0);
}