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

#ifndef MODEL_PREDICTION_HPP
#define MODEL_PREDICTION_HPP
#include <state_specification.hpp>
#include <constants.hpp>
#include <fk_vereshchagin.hpp>
#include <Eigen/Geometry>
#include <kdl_eigen_conversions.hpp>
#include <iostream>
#include <sstream>
#include <fstream>
#include <unistd.h>
#include <cmath>
#include <stdlib.h>     /* abs */

enum integration_method 
{
    PREDICTOR_CORRECTOR = 0,
    SYMPLECTIC_EULER = 1
};

class model_prediction
{
	public:
		model_prediction(const KDL::Chain &robot_chain);
		~model_prediction(){};
		
		// Used for checking joint limits
		void integrate_joint_space(
							const state_specification &current_state,
							std::vector<state_specification> &predicted_states,
							const double dt_sec, const int num_of_steps,
							const int method, const bool fk_required,
                            const bool recompute_acceleration);

		// Used for predicting future deviation from the goal state
		void integrate_cartesian_space(const state_specification &current_state,
                                       state_specification &predicted_state,
                                       const double dt_sec, 
									   const int num_of_steps);

		void integrate_to_velocity(const double &acceleration, 
								   const double &current_velocity,
								   double &predicted_velocity,
								   const int method,
							       const double dt_sec);

		void integrate_to_position(const double &acceleration,
								   const double &predicted_velocity, 
								   const double &current_position,
								   double &predicted_position,
								   const int method,
								   const double dt_sec);
		/**
		 * Calculate logarithmic map given rotation matrix. 
		 * Sources - Combined from: 
		 * http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToAngle/index.htm
		 * https://github.com/NxRLab/ModernRobotics/blob/master/packages/MATLAB/mr/MatrixLog3.m
		*/					   
		Eigen::Vector3d log_map_so3(const KDL::Rotation &matrix);
		
		/**
		 * Calculate exponential map for angular part of the given screw twist. 
		 * Given twist vector should NOT be normalized!
		*/
		KDL::Rotation exp_map_so3(const KDL::Twist &current_twist,
                                  const double rot_norm);
		/**
		 * Calculate exponential map for linear part of the given screw twist.
		 * Given twist vector should NOT be normalized!
		*/
		KDL::Vector exp_map_r3(const KDL::Twist &current_twist,
                                   const double rot_norm);
		/**
		 * Calculate exponential map for both linear and angular parts 
		 * of the given screw twist
		*/
		KDL::Frame exp_map_se3(const KDL::Twist &current_twist, 
							   const double rot_norm);

    private:
		const int NUMBER_OF_JOINTS_;
		const int NUMBER_OF_SEGMENTS_;
	    const int NUMBER_OF_FRAMES_;
	    const int NUMBER_OF_CONSTRAINTS_;

		KDL::FK_Vereshchagin fk_vereshchagin_;

		// Temp varible required for saving intermediate state,
		// if multi-step integration requirested
		state_specification temp_state_;
		KDL::Frame temp_pose_;

		// For saving prediction DATA, necessary for visualization
		const std::string CURRENT_POSE_DATA_PATH_;
		const std::string TWIST_DATA_PATH_;
		const std::string PREDICTED_POSE_DATA_PATH_;

		std::ofstream current_pose_data_file_;
		std::ofstream predicted_pose_data_file_;
		std::ofstream twist_data_file_;

    	// Help functions
		void save_pose_to_file(std::ofstream &pose_data_file, 
                               const KDL::Frame &pose);
		void save_twist_to_file(std::ofstream &twist_data_file, 
                                const KDL::Twist &twist);

		// Functions required for 3D pose itegration
		/**
		 * Calculates Exponential map for both translation and rotation
		 * Takes: 
		 * 		- body-fixed twist scaled with delta time
		 * 		- current pose to be integrated
		 * Returns tranformation of the integrated (predicted) pose
		 * If rotation is too small, function will only integrate linear part
		*/
		KDL::Frame integrate_pose(const KDL::Frame &current_pose,
								  KDL::Twist &current_twist,
								  const bool rescale_rotation);
		/** 
		 * Perform parameterization of rot twist if the angle is > PI 
		 * to avoid singularties in exponential maps.
		 * Code based on: 
		 * F. Sebastian Grassia, "Practical Parameterization of Rotations 
		 * Using the Exponential Map" paper.
		*/
		bool rescale_angular_twist(KDL::Vector &rot_twist, double &theta);

		//Converts a 3D vector to an skew matrix representation
		KDL::Rotation skew_matrix(const KDL::Vector &vector);
		//Scale a 3x3 matrix with a scalar number
		KDL::Rotation scale_matrix(const KDL::Rotation &matrix,
                                   const double scale);
		// Performs element-wise additions of two matrices
		KDL::Rotation matrix_addition(const KDL::Rotation &matrix_1,
                                      const KDL::Rotation &matrix_2);
		/** 
		 * Solving Generalized/constrained Procrustes problem i.e. 
		 * bringing computed matrix back to the SO(3) manifold.
		 * source: https://ieeexplore.ieee.org/document/88573
		*/
		void orthonormalize_rot_matrix(KDL::Rotation &rot_matrix);
		// Determine if a matrix is rotational or not
		bool is_rotation_matrix(const KDL::Rotation &m);
		// Compute determinant of a 3x3 matrix
		double determinant(const KDL::Rotation &m);
		// Compute distance of the matrix from the SO(3) manifold
		double distance_to_so3(const Eigen::Matrix3d &matrix);
		
		// Forward position and velocity kinematics, from itegrated joint values
		void compute_FK(state_specification &predicted_state);
};
#endif /* MODEL_PREDICTION_HPP */