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

#ifndef MODEL_PREDICTION_HPP
#define MODEL_PREDICTION_HPP
#include <state_specification.hpp>
#include <constants.hpp>
#include <fk_vereshchagin.hpp>
#include <Eigen/Geometry>
#include <geometry_utils.hpp>
#include <kdl_eigen_conversions.hpp>
#include <iostream>
#include <sstream>
#include <fstream>
#include <unistd.h>
#include <cmath>
#include <stdlib.h>     /* abs */

class model_prediction
{
	public:
		model_prediction(const KDL::Chain &robot_chain);
		~model_prediction(){};
		
		// Used for predicting future deviation from the goal state
		void integrate_cartesian_space(const state_specification &current_state,
                                       state_specification &predicted_state,
                                       const double dt_sec, 
									   const int num_of_steps);

		// Used for checking joint limits
		void integrate_joint_space(
							const state_specification &current_state,
							std::vector<state_specification> &predicted_states,
							const double dt_sec, const int num_of_steps,
							const int method, const bool fk_required,
                            const bool recompute_acceleration);

		// Vector integration from joint acceleration to joint velocity
		void integrate_to_velocity(const KDL::JntArray &acceleration, 
								   const KDL::JntArray &current_velocity,
								   KDL::JntArray &predicted_velocity,
								   const int method,
								   const double dt_sec);

		// Vector integration from joint velocity to joint position/angle
		void integrate_to_position(const KDL::JntArray &acceleration,
								   const KDL::JntArray &velocity, 
								   const KDL::JntArray &current_position,
								   KDL::JntArray &predicted_position,
								   const int method,
								   const double dt_sec);
    private:
		const unsigned int NUM_OF_JOINTS_;
		const unsigned int NUM_OF_SEGMENTS_;
	    const int NUM_OF_FRAMES_;
	    const int NUM_OF_CONSTRAINTS_;
		const int END_EFF_;
		
		KDL::FK_Vereshchagin fk_vereshchagin_;

		// Temp variable required for saving intermediate state,
		// if multi-step integration requirested
		state_specification temp_state_;
		KDL::Frame temp_pose_;

		std::ofstream current_pose_data_file_;
		std::ofstream predicted_pose_data_file_;
		std::ofstream twist_data_file_;

    	// Help functions
		void save_pose_to_file(std::ofstream &pose_data_file, 
                               const KDL::Frame &pose);
		void save_twist_to_file(std::ofstream &twist_data_file, 
                                const KDL::Twist &twist);

		/**
		 * Calculates Exponential map for both translation and rotation
		 * Input: 
		 * 		- current pose to be integrated
		 * 		- "body-fixed" twist scaled with delta time
		 * 		- flag for rescaling rotation if its out of 0 - PI range
		 * 		- flag for decoupling integration of linear and angular parts  
		 * Returns tranformation of the integrated (predicted) pose
		*/
		KDL::Frame integrate_pose(const KDL::Frame &current_pose,
								  KDL::Twist &current_twist,
								  const bool rescale_rotation,
								  const bool decouple_dimensions);
		
		// Forward position and velocity kinematics, from integrated joint values
		void compute_FK(state_specification &predicted_state);
};
#endif /* MODEL_PREDICTION_HPP */