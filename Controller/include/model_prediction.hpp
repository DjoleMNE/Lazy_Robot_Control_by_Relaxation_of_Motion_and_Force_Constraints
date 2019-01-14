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
#include <controller_constants.hpp>
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
		~model_prediction(){}
		
		// Used for checking joint limits
		void integrate_joint_space(
							const state_specification &current_state,
							std::vector<state_specification> &predicted_states,
							const double step_size,	const int number_of_steps,
							const int method, const bool fk_requested,
                            const bool recompute_acceleration);

		// Used for predicting future deviation from the goal state
		void integrate_cartesian_space(
							const state_specification &current_state,
							std::vector<state_specification> &predicted_states,
							const double step_size, const int number_of_steps,
							const int method, const bool recompute_acceleration);

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

		// Temp varible required for saving intermediate state,
		// if multi step integration requirested
		state_specification temp_state_;
		
		/* Workaround KDL's requirement for specifying FK VEl solver first
		input/argument as JntArrayVel. Due to this, 
		in each iteration of integration loop new JntArrayVel instance, 
		which is not real time operations. Its better to create one
		in the begging and just update its values in the loop */
		KDL::JntArrayVel temp_jntarrayvel_; 
		KDL::FrameVel temp_framevel_ = KDL::FrameVel::Identity();

		KDL::ChainFkSolverPos_recursive fk_position_solver_;
		KDL::ChainFkSolverVel_recursive fk_velocity_solver_;
		
		// Forward position and velocity kinematics, from itegrated values
		void compute_FK(state_specification &predicted_state);
};
#endif /* MODEL_PREDICTION_HPP */