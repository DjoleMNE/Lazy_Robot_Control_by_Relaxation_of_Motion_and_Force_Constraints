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
#include <iostream>
#include <sstream>
#include <time.h>
#include <fstream>
#include <unistd.h>
#include <cmath>
#include <stdlib.h>     /* abs */

class model_prediction
{
	public:
 		double time_horizon_;

		model_prediction(const KDL::Chain &robot_chain);
		~model_prediction(){}
		
		// Used for checking joint limits
		void integrate_joint_space(const state_specification &current_state,
								   state_specification &predicted_state,
								   const double step_size,
								   const int number_of_steps);

		// Used for predicting future deviation from the goal state
		void integrate_cartesian_space(const state_specification &current_state,
									   state_specification &predicted_state,
									   const double step_size,
									   const int number_of_steps);
    private:
		KDL::Chain robot_chain_;

		const int NUMBER_OF_SEGMENTS_;
		const int NUMBER_OF_JOINTS_;
	    const int NUMBER_OF_FRAMES_;

		KDL::ChainFkSolverPos_recursive fk_position_solver_;
		KDL::ChainFkSolverVel_recursive fk_velocity_solver_;

		/* Workaround KDL's requirement for specifying FK VEl solver first
		input/argument as JntArrayVel. Due to this, 
		in each iteration of integration loop new JntArrayVel instance, 
		which is not real time operations. Its better to create one
		in the begging and just update its values in the loop */
		KDL::JntArrayVel temp_jntarrayvel; 
};
#endif /* MODEL_PREDICTION_HPP */