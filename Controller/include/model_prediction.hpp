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
							const double step_size,	const int number_of_steps,
							const int method, const bool fk_required,
                            const bool recompute_acceleration);

		// Used for predicting future deviation from the goal state
		KDL::Frame integrate_cartesian_space(const KDL::Frame &current_pose,
                            		   const KDL::Twist &current_twist,
                                       const double dt, 
									   const int number_of_steps);

		void integrate_to_velocity(const double &acceleration, 
								   const double &current_velocity,
								   double &predicted_velocity,
								   const int method,
							       const double dt);

		void integrate_to_position(const double &acceleration,
								   const double &predicted_velocity, 
								   const double &current_position,
								   double &predicted_position,
								   const int method,
								   const double dt);
    private:
		const int NUMBER_OF_JOINTS_;
		const int NUMBER_OF_SEGMENTS_;
	    const int NUMBER_OF_FRAMES_;
	    const int NUMBER_OF_CONSTRAINTS_;

		int fk_solver_result_;
		KDL::FK_Vereshchagin fk_vereshchagin_;

		// Temp varible required for saving intermediate state,
		// if multi-step integration requirested
		double x_, y_, z_, w_;
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
		void normalize_rot_matrix(KDL::Rotation &rot_martrix);

		// Forward position and velocity kinematics, from itegrated joint values
		void compute_FK(state_specification &predicted_state);

};
#endif /* MODEL_PREDICTION_HPP */