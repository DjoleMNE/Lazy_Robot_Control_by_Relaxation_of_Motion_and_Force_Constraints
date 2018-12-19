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

#ifndef DYNAMICS_CONTROLLER_HPP_
#define DYNAMICS_CONTROLLER_HPP_
#include <solver_vereshchagin.hpp>
#include <state_specification.hpp>
#include <youbot_driver/youbot/YouBotManipulator.hpp>
#include <iostream>
#include <sstream>
#include <time.h>
#include <boost/assign/list_of.hpp>
#include <fstream>
#include <unistd.h>
#include <cmath>
#include <stdlib.h>     /* abs */

class dynamics_controller
{
  public:
    dynamics_controller(const KDL::Chain &chain,
                        const KDL::Twist &root_acc,
                        std::vector<double> joint_position_limits,
                        std::vector<double> joint_velocity_limits,
                        std::vector<double> joint_acceleration_limits,
                        std::vector<double> joint_torque_limits,
                        double rate_hz);
    ~dynamics_controller();

    int control(youbot::YouBotManipulator &robot, bool simulation_environment);

    void reset_desired_state();

    //Methods for defining robot task via 3 interfaces
    void define_ee_constraint_task(const std::vector<bool> constraint_direction,
                              const std::vector<double> cartesian_acceleration);
    void define_ee_external_force_task(const std::vector<double> external_force);
    void define_feadforward_torque_task(const std::vector<double> ff_torque);

  private:
    double rate_hz_;
    double dt_;

    const int NUMBER_OF_JOINTS_;
	const int NUMBER_OF_SEGMENTS_;
	const int NUMBER_OF_FRAMES_;
	const int NUMBER_OF_CONSTRAINTS_;

    const std::vector<double> joint_position_limits_;
    const std::vector<double> joint_velocity_limits_;
    const std::vector<double> joint_acceleration_limits_;
    const std::vector<double> joint_torque_limits_;

    KDL::Solver_Vereshchagin hd_solver_;
    KDL::Chain robot_chain_;
	
    state_specification robot_state_;
    state_specification commands_;
    state_specification desired_state_;
    state_specification predicted_state_;

    void reset_state(state_specification &state);
    void check_limits(state_specification &state);
    void update_task();
    void update_current_state();
    void evaluate_dynamics();

    void set_ee_constraints(state_specification &state,
                            const std::vector<bool> constraint_direction,
                            const std::vector<double> cartesian_acceleration);
    void set_external_forces(state_specification &state, 
                             const std::vector<double> external_force);
    void set_feadforward_torque(state_specification &state,
                                const std::vector<double> ff_torque);    
};
#endif /* DYNAMICS_CONTROLLER_HPP_*/