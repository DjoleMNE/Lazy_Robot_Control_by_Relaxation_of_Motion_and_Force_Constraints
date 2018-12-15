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
#include <command_specification.hpp>
#include <task_specification.hpp>
#include <iostream>
#include <sstream>
#include <time.h>
#include <boost/assign/list_of.hpp>
#include <fstream>
#include <unistd.h>
#include <cmath>
#include <stdlib.h>     /* abs */

class model_prediction
{
	public:
		model_prediction(KDL::Chain &arm_chain);
		~model_prediction(){}
		
		// Write integrated values in state variables
		void integrate(const state_specification &current_state,
						state_specification &predicted_state,
						const double step_size,
						const int number_of_steps = 1);

		// Write integrated values in command variables
		void integrate(state_specification &current_state,
						command_specification &predicted_state,
						const double &step_size,
						const int &number_of_steps = 1);
                        
    private:
        double time_horizon_;
		KDL::Chain arm_chain_;
		KDL::ChainFkSolverPos_recursive fk_position_solver_;
		KDL::ChainFkSolverVel_recursive fk_velocity_solver_;
};
#endif /* MODEL_PREDICTION_HPP */