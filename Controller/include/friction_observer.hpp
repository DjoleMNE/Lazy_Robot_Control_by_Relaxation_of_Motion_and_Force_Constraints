/*
Author(s): Djordje Vukcevic <djordje dot vukcevic at h-brs dot de>
Institute: Hochschule Bonn-Rhein-Sieg

Copyright (c) [2021]

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

#ifndef FRICTION_OBSERVER_HPP
#define FRICTION_OBSERVER_HPP
#include <constants.hpp>
#include <Eigen/Core>
#include <kdl_eigen_conversions.hpp>
#include <kdl/frames_io.hpp>
#include <iostream>
#include <sstream>
#include <fstream>
#include <unistd.h>
#include <cmath>
#include <stdlib.h>

enum friction_observer_type 
{
    PD = 0,
    PID = 1
};

/**
 * \brief Model-free observer for the estimation of friction torque in robot's joints, using the measurements from a real torque-sensor.
 *
 * Implementation based on:
 * M. J. Kim, F. Beck, C. Ott and A. Albu-Schäffer,
 * "Model-Free Friction Observers for Flexible Joint Robots With Torque Measurements",
 * in IEEE Transactions on Robotics, vol. 35(6), 2019.
 */
class FrictionObserver
{
public:
    /**
     * Constructor for the estimator, it will allocate all the necessary memory
     * \param num_joints Number of joints in which friction should be estimated
     * \param rotor_inertia Scalar value for the joint rotor's inertia (before or after gears?)
     * \param gain_l Observer's derivative-gain for each joint
     * \param gain_lp Observer's proportional-gain for each joint
     * \param gain_li Observer's integral-gain for each joint
     * \param observer_type Desired type of observer: PD or PID
     * \param integration_method Desired method to integrate motion: Symplectic Euler or Trapezoidal rule
     * \param state_integration_resent_count number of estimation cycles after which it is necessary to rest the integrated state so to avoid drift errors. To disable this re-set option, set count to 0.
     * \param filter_constant Parameter defining how much an estimated signal should be filtered by the low-pass filter.
     *                        This input value should be between 0 and 1. Higher the number means more noise needs to be filtered-out.
     *                        The filter can be turned off by setting this value to 0.
     * \param dt_sec Time-period at which user updates this estimation loop (in seconds).
     */
    FrictionObserver(const int num_joints, const double dt_sec, const Eigen::VectorXd &rotor_inertia, const std::vector<double> &joint_velocity_limits,
                     const Eigen::VectorXd &gain_l, const Eigen::VectorXd &gain_lp, const Eigen::VectorXd &gain_li,
                     const int observer_type = friction_observer_type::PD, const int integration_method = integration_method::SYMPLECTIC_EULER,
                     const int state_integration_resent_count = 0, const double filter_constant = 0.0);
    ~FrictionObserver(){};

    // Set intially measured joint state (values should correspond to those before the gears)
    int setInitialState(const KDL::JntArray &motor_position, const KDL::JntArray &motor_velocity);

    /**
     * This method estimates the friction torque in robot's joints.
     * Input parameters:
     * \param motor_position The current (measured) motor positions
     * \param motor_velocity The current (measured) motor velocities
     * \param joint_torque_cmd The friction-free command torques computed by the controller.
     * \param joint_torque_measured The joint torques measured by the sensors located on the output shaft of joints.
     *
     * Output parameters:
     * \param observed_joint_friction The estimated joint friction torque that should be compensated by the outer controller, in the current control cycle.
     *
     * @return error/success code
     */ 
    int estimateFrictionTorque(const KDL::JntArray &motor_position, const KDL::JntArray &motor_velocity, const KDL::JntArray &joint_torque_cmd, const KDL::JntArray &joint_torque_measured, KDL::JntArray &observed_joint_friction);

    /** 
     * Get predicted nominal (friction-free) state: latest-integrated state
     * Output parameters:
     * \param nominal_motor_position  The estimated motor position that should be used by the outer controller instead of the measured position, in the next control cycle.
     * \param nominal_motor_velocity  The estimated motor velocity that should be used by the outer controller instead of the measured velocity, in the next control cycle.
     */
    void getNominalState(KDL::JntArray &nominal_motor_position, KDL::JntArray &nominal_motor_velocity);

private:
    const double DT_SEC, FILTER_CONST;
    const int INTEGRATION_METHOD, OBSERVER_TYPE;
    const unsigned int NUM_JOINTS, STATE_INTEGRATION_RESET_COUNT;
    const Eigen::VectorXd ROTOR_INERTIA, GAIN_L, GAIN_LP, GAIN_LI, COMMON_GAIN;
    const std::vector<double> JOINT_VEL_LIMIT;

    unsigned int integration_count;

    Eigen::VectorXd motor_flat_position, previous_motor_position, theta_nominal, theta_dot_nominal, theta_dot_dot_nominal, error_nominal, error_dot_nominal, error_integral, filtered_friction, estimated_friction;
};

#endif /* FRICTION_OBSERVER_HPP */
