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

#ifndef TASK_SPECIFICATION_HPP
#define TASK_SPECIFICATION_HPP
#include <state_specification.hpp>

class task_specification
{
	public:
		task_specification(){}
		~task_specification(){}

		KDL::JntArray feedforward_torque;
		KDL::Wrenches external_force;
		KDL::Jacobian ee_unit_constraint_forces;
		KDL::JntArray ee_acceleration_energy;
		
		void init_state(int number_of_joints,
						int number_of_segments,
						const int NUMBER_OF_CONSTRAINTS)
		{
			feedforward_torque.resize(number_of_joints);
			external_force.resize(number_of_segments);
			ee_unit_constraint_forces.resize(NUMBER_OF_CONSTRAINTS); //alpha
			ee_acceleration_energy.resize(NUMBER_OF_CONSTRAINTS); //beta
		}
};
#endif /* TASK_SPECIFICATION_HPP */