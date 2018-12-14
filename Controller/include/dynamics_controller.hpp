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
#include <command_specification.hpp>
#include <youbot_driver/youbot/YouBotManipulator.hpp>

class dynamics_controller
{
  public:
      dynamics_controller(
          const KDL::Chain &chain,
          const KDL::Twist &root_acc,
          std::vector<double> &joint_position_limits,
          std::vector<double> &joint_velocity_limits,
          std::vector<double> &joint_acceleration_limits,
          std::vector<double> &joint_torque_limits,
          double rate_hz);
      ~dynamics_controller();

      int control(youbot::YouBotManipulator &robot);
      void reset_desired_state();
      void set_ee_constraints(std::vector<bool> &constraint_direction, 
                              std::vector<double> &cartesian_acceleration,
                              state_specification &state);
      void set_external_forces();
      void set_feadforward_torque();
    
      double rate_hz_ = 0.0;
      double dt_;

  private:
    void integrate_robot_motion(state_specification current_state,
                                state_specification predicted_state,
                                int number_of_steps);
    
    void reset_state(state_specification state);
    void check_limits(state_specification state);

    const int NUMBER_OF_CONSTRAINTS = 6;
    const KDL::Chain &chain_;
    int number_of_frames_;
    int number_of_joints_;
    int number_of_segments_;

    KDL::Solver_Vereshchagin hd_solver_;
    state_specification desired_state_;
    state_specification current_state_;
    state_specification predicted_state_;

    std::vector<double> joint_position_limits_;
    std::vector<double> joint_velocity_limits_;
    std::vector<double> joint_acceleration_limits_;
    std::vector<double> joint_torque_limits_;
};
#endif /* DYNAMICS_CONTROLLER_HPP_*/