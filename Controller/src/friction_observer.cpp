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

#include <friction_observer.hpp>

FrictionObserver::FrictionObserver(const int num_joints, const double dt_sec, const Eigen::VectorXd &rotor_inertia, const Eigen::VectorXd &gain_l,
                                   const Eigen::VectorXd &gain_lp, const Eigen::VectorXd &gain_li, const int observer_type,
                                   const int integration_method, const int state_integration_resent_count, const double filter_constant):
    DT_SEC(dt_sec), FILTER_CONST(filter_constant),
    INTEGRATION_METHOD(integration_method), OBSERVER_TYPE(observer_type),
    NUM_JOINTS(num_joints), STATE_INTEGRATION_RESET_COUNT(state_integration_resent_count),
    ROTOR_INERTIA(rotor_inertia), GAIN_L(gain_l), GAIN_LP(gain_lp), GAIN_LI(gain_li), COMMON_GAIN(-ROTOR_INERTIA.cwiseProduct(GAIN_L)),
    integration_count(0), theta_nominal(NUM_JOINTS), theta_dot_nominal(NUM_JOINTS), theta_dot_dot_nominal(NUM_JOINTS),
    error_nominal(NUM_JOINTS), error_dot_nominal(NUM_JOINTS),
    error_integral(NUM_JOINTS), filtered_friction(NUM_JOINTS), estimated_friction(NUM_JOINTS)
{
    assert(ROTOR_INERTIA.rows() == NUM_JOINTS);
    assert(GAIN_L.rows()        == NUM_JOINTS);
    assert(GAIN_LP.rows()       == NUM_JOINTS);
    assert(GAIN_LI.rows()       == NUM_JOINTS);

    // Control loop frequency must be between 1 and 1000Hz (limited by Kinova's low-level torque interface)
    assert(("Selected frequency is too low", 1 <= 1.0 / DT_SEC));
    assert(("Selected frequency is too high", 1.0 / DT_SEC <= 1000));

    // Low-pass filter constant must be between 0 and 1
    assert(("Selected filter constant is too low", 0 <= FILTER_CONST));
    assert(("Selected filter constant is too high", FILTER_CONST <= 1));

    for (unsigned int i = 0; i < NUM_JOINTS; i++)
    {
        assert(("Incorrect selection of gains", GAIN_LP(i) > 0.0 && GAIN_LI(i) > 0.0 && GAIN_L(i) > 0.0));

        // For this observer to work, condition Lp^2 > 2Li has to be satisfied
        if (OBSERVER_TYPE == friction_observer_type::PID) assert(("Incorrect scale of proportional and integral gains", GAIN_LP(i) * GAIN_LP(i) > 2 * GAIN_LI(i)));
    }
}

// Set intially measured joint state
int FrictionObserver::setInitialState(const KDL::JntArray &motor_position, const KDL::JntArray &motor_velocity)
{
    if (motor_position.rows() != NUM_JOINTS || motor_velocity.rows() != NUM_JOINTS) return -1;

    theta_nominal     = motor_position.data;
    theta_dot_nominal = motor_velocity.data;

    error_integral.setZero();
    filtered_friction.setZero();

    return 0;
}

// Estimates the friction torque in robot's joints
int FrictionObserver::estimateFrictionTorque(const KDL::JntArray &motor_position, const KDL::JntArray &motor_velocity, const KDL::JntArray &joint_torque_cmd,
                                             const KDL::JntArray &joint_torque_measured, KDL::JntArray &observed_joint_friction)
{
    /**
     * ====================================================================================
     * Implementation based on:
     * M. J. Kim, F. Beck, C. Ott and A. Albu-Schäffer,
     * "Model-Free Friction Observers for Flexible Joint Robots With Torque Measurements",
     * in IEEE Transactions on Robotics, vol. 35(6), 2019.
     * ====================================================================================
     */

    // Check vector sizes
    if (motor_position.rows() != NUM_JOINTS || motor_velocity.rows() != NUM_JOINTS || joint_torque_cmd.rows() != NUM_JOINTS || joint_torque_measured.rows() != NUM_JOINTS || observed_joint_friction.rows() != NUM_JOINTS) return -1;

    // Reset the state integration
    if (STATE_INTEGRATION_RESET_COUNT != 0 && integration_count > STATE_INTEGRATION_RESET_COUNT)
    {
        setInitialState(motor_position, motor_velocity);
        integration_count = 0;
    }

    // Nominal acceleration: (difference between commanded and measured torque) times rotor inertia
    theta_dot_dot_nominal = (joint_torque_cmd.data - joint_torque_measured.data).cwiseProduct(ROTOR_INERTIA);

    // Solve (integrate) the second-order ODE (motor acceleration) to calculate nominal motor velocity and position
    if (INTEGRATION_METHOD == integration_method::SYMPLECTIC_EULER)
    {
        theta_dot_nominal += theta_dot_dot_nominal * DT_SEC;
        theta_nominal     += theta_dot_nominal * DT_SEC; // Symplectic Euler method
    }
    else if (INTEGRATION_METHOD == integration_method::PREDICTOR_CORRECTOR)
    {
        theta_nominal     += (theta_dot_nominal - theta_dot_dot_nominal * DT_SEC / 2.0) * DT_SEC; // Trapezoidal method
        theta_dot_nominal += theta_dot_dot_nominal * DT_SEC;
    }
    else return -1;

    // Calculate state error
    error_nominal     = theta_nominal     - motor_position.data; // position error
    error_dot_nominal = theta_dot_nominal - motor_velocity.data; // velocity error

    // Calculate friction estimate: PD or PID type
    if      (OBSERVER_TYPE == friction_observer_type::PD) estimated_friction = COMMON_GAIN.cwiseProduct(error_dot_nominal + GAIN_LP.cwiseProduct(error_nominal));
    else if (OBSERVER_TYPE == friction_observer_type::PID)
    {
        error_integral += error_nominal * DT_SEC;
        estimated_friction = COMMON_GAIN.cwiseProduct(error_dot_nominal + GAIN_LP.cwiseProduct(error_nominal) + GAIN_LI.cwiseProduct(error_integral));
    }

    // First order low-pass filter: filter out the noise from the estimated signal. This filter can be turned off by setting FILTER_CONST value to 0
    filtered_friction = FILTER_CONST * filtered_friction + (1.0 - FILTER_CONST) * estimated_friction;

    // Write function output
    observed_joint_friction.data = filtered_friction;

    return 0;
}

// Get predicted nominal (friction-free) state: latest-integrated state
void FrictionObserver::getNominalState(KDL::JntArray &nominal_motor_position, KDL::JntArray &nominal_motor_velocity)
{
    assert(nominal_motor_position.rows() == NUM_JOINTS);
    assert(nominal_motor_velocity.rows() == NUM_JOINTS);

    nominal_motor_position.data = theta_nominal;
    nominal_motor_velocity.data = theta_dot_nominal;
}
