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
#define EPSILON 0.2// margin to allow for rounding errors
#define MIN_ANGLE 1e-6

model_prediction::model_prediction(const KDL::Chain &robot_chain): 
    NUMBER_OF_JOINTS_(robot_chain.getNrOfJoints()),
    NUMBER_OF_SEGMENTS_(robot_chain.getNrOfSegments()),
    NUMBER_OF_FRAMES_(robot_chain.getNrOfSegments() + 1),
    NUMBER_OF_CONSTRAINTS_(dynamics_parameter::NUMBER_OF_CONSTRAINTS),
    fk_vereshchagin_(robot_chain),
    temp_state_(NUMBER_OF_JOINTS_, NUMBER_OF_SEGMENTS_, 
                NUMBER_OF_FRAMES_, NUMBER_OF_CONSTRAINTS_),
    temp_pose_(KDL::Frame::Identity()),
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
    
    KDL::Twist body_fixed_twist; 
    
    body_fixed_twist(0) = 0.5;
    body_fixed_twist(1) = 0.0;
    body_fixed_twist(2) = 0.0;

    body_fixed_twist(3) = 0.0;
    body_fixed_twist(4) = M_PI/2;
    body_fixed_twist(5) = 0.0;

    body_fixed_twist = temp_pose_.M.Inverse(body_fixed_twist) * dt_sec;

    // body_fixed_twist = \
    //     temp_pose_.M.Inverse(current_state.frame_velocity[NUMBER_OF_SEGMENTS_ - 1]) * dt_sec;
    orthonormalize_rot_matrix(temp_pose_.M);
    assert(("Current rotation matrix", is_rotation_matrix(temp_pose_.M)));
    
    for (int i = 0; i < num_of_steps; i++)
    {
        temp_pose_ = integrate_pose(temp_pose_, body_fixed_twist, true);
        orthonormalize_rot_matrix(temp_pose_.M);
        assert(("Integrated rotation matrix", is_rotation_matrix(temp_pose_.M)));
    }
    
    #ifndef NDEBUG
        std::cout << "Measured End-effector Position:\n" 
                  << current_state.frame_pose[NUMBER_OF_SEGMENTS_ - 1].p 
                  << std::endl;

        std::cout << "Measured End-effector Orientation:\n" 
                  << current_state.frame_pose[NUMBER_OF_SEGMENTS_ - 1].M 
                  << std::endl;

        std::cout << std::endl;
        
        std::cout << "Integrated End-effector Position 1:\n" 
                  << temp_pose_.p  << std::endl;
        
        std::cout << "Integrated End-effector Orientation 1:\n" 
                  << temp_pose_.M  << std::endl;

        std::cout << "Delta Angle: " << body_fixed_twist.rot.Norm() << std::endl; 

        twist_data_file_.open(TWIST_DATA_PATH_);
        save_twist_to_file(twist_data_file_, body_fixed_twist);

        current_pose_data_file_.open(CURRENT_POSE_DATA_PATH_);
        save_pose_to_file(current_pose_data_file_, 
                          current_state.frame_pose[NUMBER_OF_SEGMENTS_ - 1]);

        predicted_pose_data_file_.open(PREDICTED_POSE_DATA_PATH_);
        save_pose_to_file(predicted_pose_data_file_, temp_pose_);
    #endif
    
    predicted_state.frame_pose[NUMBER_OF_SEGMENTS_ - 1] = temp_pose_;
}

// Calculate  exponential map for angular part of the given screw twist
KDL::Rotation model_prediction::exp_map_so3(const KDL::Twist &current_twist,
                                            const double rot_norm)
{   
    // First normalize given twist vector!
    return KDL::Rotation::Rot2(current_twist.rot / rot_norm, rot_norm);
}

// Calculate exponential map for linear part of the given screw twist
// Given twist vector should NOT be normalized!
KDL::Vector model_prediction::exp_map_r3(const KDL::Twist &current_twist,
                                         const double rot_norm)
{
    // Convert rotation vector to a skew matrix 
    KDL::Rotation skew_rotation = skew_matrix( current_twist.rot );
    KDL::Rotation skew_rotation_square = skew_rotation * skew_rotation;
    double rot_norm_square = rot_norm * rot_norm;

    return (matrix_addition(KDL::Rotation::Identity(),
                            matrix_addition(scale_matrix(skew_rotation, 
                                                         (1 - cos(rot_norm)) / rot_norm_square
                                                        ),
                                            scale_matrix(skew_rotation_square, 
                                                         (1 - sin(rot_norm) / rot_norm) / rot_norm_square
                                                        )
                                           )
                           )
           ) * current_twist.vel;
}

/**
 * Calculate exponential map for both linear and angular parts 
 * of the given screw twist
*/
KDL::Frame model_prediction::exp_map_se3(const KDL::Twist &current_twist,
                                         const double rot_norm)
{   
    return KDL::Frame(exp_map_so3(current_twist, rot_norm),
                      exp_map_r3(current_twist, rot_norm));
}

/**
 * Code is based on formulas given in books:
 * "Modern Robotics", F.C.Park
 * "Robot Kinematics and Dynamics", Herman B.
*/
KDL::Frame model_prediction::integrate_pose(const KDL::Frame &current_pose,
                                            KDL::Twist &current_twist, 
                                            const bool rescale_rotation) 
{
    double rot_norm;
    if(rescale_rotation) rescale_angular_twist(current_twist.rot, rot_norm);
    else rot_norm = current_twist.rot.Norm();

    if (rot_norm < MIN_ANGLE) {
        return current_pose * KDL::Frame(KDL::Rotation::Identity(), current_twist.vel);
    } else {
        return current_pose * exp_map_se3(current_twist, rot_norm);
    }
}

/** 
 * Perform parameterization of rot twist if the angle is > PI 
 * to avoid singularties in exponential maps.
 * Re-Parameterize to a rotation of (2PI - theta) about the opposite axis 
 * when angle gets too close to 2PI.
 * Code based on: F. Sebastian Grassia, "Practical Parameterization of Rotations 
 * Using the Exponential Map" paper.
*/
bool model_prediction::rescale_angular_twist(KDL::Vector &rot_twist, double &theta)
{
    bool is_parameterized = false;
    theta = rot_twist.Norm();

    if (theta > M_PI){
        double scale = theta;
        if (theta > 2 * M_PI){/* first get theta into range 0..2PI */
            theta = std::fmod(theta, 2 * M_PI);
            scale = theta / scale;
            rot_twist = rot_twist * scale;
            is_parameterized = true;
        }
        if (theta > M_PI){
            scale = theta;
            theta = 2 * M_PI - theta;
            scale = 1.0 - 2 * M_PI / scale;
            rot_twist = rot_twist * scale;
            is_parameterized = true;
        }
    }
    return is_parameterized;
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
                                             const double scale)
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

/** 
 * Solving Generalized/constrained Procrustes problem i.e. 
 * bringing computed matrix back to the SO(3) manifold: re-orthonormalization.
 * source: https://ieeexplore.ieee.org/document/88573
*/
void model_prediction::orthonormalize_rot_matrix(KDL::Rotation &rot_matrix)
{
    Eigen::Matrix3d eigen_matrix = rotation_to_eigen(rot_matrix);

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(eigen_matrix,
                                          Eigen::ComputeFullU | Eigen::ComputeFullV);
    eigen_matrix = svd.matrixU() * svd.matrixV().transpose();

    if(eigen_matrix.determinant() <= 0.0)
    {
        Eigen::Matrix3d singular_values = Eigen::Matrix3d::Identity(3, 3);
        singular_values(2, 2) = -1;
        eigen_matrix = svd.matrixU() * singular_values * svd.matrixV().transpose();
    }

#ifndef NDEBUG
    Eigen::JacobiSVD<Eigen::Matrix3d> svd1(eigen_matrix,
                                          Eigen::ComputeFullU | Eigen::ComputeFullV);   
    std::cout << "Singular values: " << svd1.singularValues().transpose() << std::endl;
    std::cout << "Determinant: " << eigen_matrix.determinant() << std::endl;
    std::cout << "Distance to SO(3): " << distance_to_so3(eigen_matrix) << std::endl;
#endif
    rot_matrix = eigen_to_rotation(eigen_matrix);
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
* Source: 
*   1. "Modern Robotics" book code on Github
*   2. http://www.euclideanspace.com/maths/algebra/matrix/orthogonal/rotation/
*/
bool model_prediction::is_rotation_matrix(const KDL::Rotation &m)
{
    Eigen::Matrix3d eigen_matrix = rotation_to_eigen(m);
    if(eigen_matrix.determinant() < (1 - MIN_ANGLE)) return false;
    if(distance_to_so3(eigen_matrix) > MIN_ANGLE) return false;
    return true;
	// if (std::fabs(m(0, 0)*m(0, 1) + m(0, 1)*m(1, 1) + m(0, 2)*m(1, 2)    ) > EPSILON) return false;
	// if (std::fabs(m(0, 0)*m(2, 0) + m(0, 1)*m(2, 1) + m(0, 2)*m(2, 2)    ) > EPSILON) return false;
	// if (std::fabs(m(1, 0)*m(2, 0) + m(1, 1)*m(2, 1) + m(1, 2)*m(2, 2)    ) > EPSILON) return false;
	// if (std::fabs(m(0, 0)*m(0, 0) + m(0, 1)*m(0, 1) + m(0, 2)*m(0, 2) - 1) > EPSILON) return false;
	// if (std::fabs(m(1, 0)*m(1, 0) + m(1, 1)*m(1, 1) + m(1, 2)*m(1, 2) - 1) > EPSILON) return false;
	// if (std::fabs(m(2, 0)*m(2, 0) + m(2, 1)*m(2, 1) + m(2, 2)*m(2, 2) - 1) > EPSILON) return false;
	// return (std::fabs(determinant(m) - 1) < EPSILON);
}

/**
 * Code is defined here:
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

/**
 * Compute distance of the matrix from the SO(3) manifold
 * Source "Modern robotics" Book. Code is defined here:
 * https://github.com/NxRLab/ModernRobotics/blob/master/packages/MATLAB/mr/DistanceToSO3.m
*/
double model_prediction::distance_to_so3(const Eigen::Matrix3d &matrix) 
{
    /**
     * Returns the Frobenius norm to describe the distance of matrix from the
     * SO(3) manifold
        :param matrix: A 3x3 matrix
        :return: A quantity describing the distance of mat from the SO(3)
                manifold
     * Computes the distance from matrix to the SO(3) manifold using the 
     * following method:
        If det(matrix) <= 0, return a large number.
        If det(matrix) > 0, return norm(matrix^T.matrix - I).
    */
    if (matrix.determinant() > 0)
        return (matrix.transpose() * matrix - Eigen::Matrix3d::Identity(3, 3)).norm();
    else return 1E+9;
}

/**
 * Calculate logarithmic map given rotation matrix
 * Returns non-normalized axis, i.e. angle is the norm of the return vector
 * Two singularities: 0 and PI angle rotations
 * In 0 angle case, vector of zeros is returned 
 * In case of PI angle, one of the 3 vector choices is returned. 
 * More specifically, the vector corresponding to a scalar computation that
 * involves the highest value of denominator, in order to avoid numerical issues
 * occuring when a numerator is divided with a small number.
 * Sources - Combined from: 
 * http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToAngle/index.htm
 * https://github.com/NxRLab/ModernRobotics/blob/master/packages/MATLAB/mr/MatrixLog3.m
*/
KDL::Vector model_prediction::log_map_so3(const KDL::Rotation &matrix)
{
    double angle, x, y, z; // variables for result
    double epsilon1 = 0.00001; // margin to allow for rounding errors
    double epsilon2 = epsilon1 * 10; // margin to distinguish between 0 and 180 degrees

    //Check first if one of two singularity cases has occurred  
    if (   (std::fabs(matrix.data[1] - matrix.data[3]) < epsilon1)
        && (std::fabs(matrix.data[2] - matrix.data[6]) < epsilon1)
        && (std::fabs(matrix.data[5] - matrix.data[7]) < epsilon1))
    {
        // singularity found
        // first check for identity matrix which must have +1 for all terms
        // in leading diagonal and zero in other terms
        if (   (std::fabs(matrix.data[1] + matrix.data[3])                      < epsilon2)
            && (std::fabs(matrix.data[2] + matrix.data[6])                      < epsilon2)
            && (std::fabs(matrix.data[5] + matrix.data[7])                      < epsilon2)
            && (std::fabs(matrix.data[0] + matrix.data[4] + matrix.data[8] - 3) < epsilon2))
        {
            // this singularity is identity matrix so angle = 0, axis is arbitrary
            // Because we use this for error calc, its best to return 0 vector
            return KDL::Vector(0.0, 0.0, 0.0);
        }

        // Otherwise this singularity is angle of 180
        angle = M_PI;
        double xx = (matrix.data[0] + 1) / 2;
        double yy = (matrix.data[4] + 1) / 2;
        double zz = (matrix.data[8] + 1) / 2;
        double xy = (matrix.data[1] + matrix.data[3]) / 4;
        double xz = (matrix.data[2] + matrix.data[6]) / 4;
        double yz = (matrix.data[5] + matrix.data[7]) / 4;

        if ((xx > yy) && (xx > zz))
        {
            // matrix.data[0] is the largest diagonal term
            x = sqrt(xx);
            y = xy/x;
            z = xz/x;
        }
        else if (yy > zz)
        {
            // matrix.data[4] is the largest diagonal term
            y = sqrt(yy);
            x = xy/y;
            z = yz/y;
        }
        else
        {
            // matrix.data[8] is the largest diagonal term so base result on this
            z = sqrt(zz);
            x = xz/z;
            y = yz/z;
        }
        return angle * KDL::Vector(x, y, z); // return 180 deg rotation
    }

    double acos_input = (matrix.data[0] + matrix.data[4] + matrix.data[8] - 1) / 2;
    assert(acos_input < 1.0); assert(acos_input > -1.0);

    // If the matrix is slightly non-orthogonal, 
    // 'acos_input' maybe out of the (-1, +1) range.
    // Therefore, clamp it between those values to avoid NaNs.
    // Btw, if out of range happens, the first two if statements in above code 
    // are not properly doing what they are supposed to
    // So I am not sure if this clamping is necessary here
    angle = acos(std::max(-1.0, std::min(1.0, acos_input)));

#ifndef NDEBUG
    // if this even happens, epsilon above should be increased
    // or logic behind if_s to be changed
    if (std::fabs(angle) < 0.001) std::cout << "Too small angle: " << angle << std::endl;
    else if (std::fabs(angle) > M_PI - MIN_ANGLE) std::cout << "Too big angle: " << angle << std::endl;
#endif

    //If following assertions fail, above if statements are not working properly
    assert(std::fabs(angle) <  M_PI - MIN_ANGLE);
    assert(angle > -epsilon1);

    // prevent divide by zero, should not happen if matrix is orthogonal and 
    // should be caught by singularity test above, but I've left it in just in case
	if (std::fabs(angle) < epsilon1) return KDL::Vector(0.0, 0.0, 0.0);

    x = (matrix.data[7] - matrix.data[5]);
    y = (matrix.data[2] - matrix.data[6]);
    z = (matrix.data[3] - matrix.data[1]);

    /** 
     * 1 / (2 * sin(Angle)) part is the normalizing factor. 
     * Multiplying vector with this factor produces norm of 1, 
     * but the vector is additionally multiplied with the angle, 
     * such that, the finial vector that is results from this function 
     * represent a rotation that gets a frame from indentity orientation to the 
     * input orientation, in one step of time! 
     * Bassically, logarithmic map on SO(3).
    */
    return (angle / (2 * sin(angle))) * KDL::Vector(x, y, z);
}

/**
 * Calculate logarithmic map given translation vector and
 * angular twist as result of Log_SO(e). 
 * Sources - Combined from: 
 * S. Grazioso et al., "From Differential Geometry of Curves to Helical 
 * Kinematics of Continuum Robots Using Exponential Mapping"
 * https://github.com/NxRLab/ModernRobotics/blob/master/packages/MATLAB/mr/MatrixLog6.m
 * Function takes non-normalized vectors of angular twist and translation.
*/					   
KDL::Vector model_prediction::log_map_r3(const KDL::Vector &translation,
                                         const KDL::Vector &angular_twist)
{
    // Convert rotation vector to a skew matrix 
    KDL::Rotation skew_rotation = skew_matrix( angular_twist );
    KDL::Rotation skew_rotation_square = skew_rotation * skew_rotation;
    //Compute norm of angular twist, i.e. theta angle of rotation
    double theta = angular_twist.Norm();
    double theta_square = theta * theta;
     
    return matrix_addition(KDL::Rotation::Identity(), 
                           matrix_addition(scale_matrix(skew_rotation, -0.5),
                                           scale_matrix(skew_rotation_square,
                                                        (1 - (theta * cos(theta/2)) / (2 * sin(theta/2))) / theta_square)
                                          )
                           ) * translation;
}

/**
 * Calculate logarithmic map given transformation matrix.
 * Sources - Combined from: 
 * S. Grazioso et al., "From Differential Geometry of Curves to Helical 
 * Kinematics of Continuum Robots Using Exponential Mapping"
 * https://github.com/NxRLab/ModernRobotics/blob/master/packages/MATLAB/mr/MatrixLog6.m
*/					   
KDL::Twist model_prediction::log_map_se3(const KDL::Frame &pose)
{
    // First compute log on SO(3), and get norm of the angular vector
    KDL::Vector angular_twist = log_map_so3(pose.M);  
    
    // If rotation is too small, return zero angular twist and 
    // linear twist equal to translational vector of the pose
    if(angular_twist.Norm() < MIN_ANGLE)
    {
        return KDL::Twist(pose.p, KDL::Vector(0.0, 0.0, 0.0));
    }

    // Else, additionally compute log map on R3 and its result return together 
    // with already computed log on SO(3) 
    return KDL::Twist(log_map_r3(pose.p, angular_twist), angular_twist);
}
