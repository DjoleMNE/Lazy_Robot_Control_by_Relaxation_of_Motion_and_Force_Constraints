/*
Author(s): Djordje Vukcevic, Sven Schneider
Institute: Hochschule Bonn-Rhein-Sieg
Description: Mediator component for enabling conversion of data types.
Acknowledgment: This sofware component is based on Jeyaprakash Rajagopal's 
master thesis code.

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

#ifndef CONTROLLER_CONSTANTS_HPP_
#define CONTROLLER_CONSTANTS_HPP_
#include <Eigen/Core>
#include <stdlib.h> /* abs */
#include <unistd.h>
#include <cmath>

#define PI 3.14159265358979323846
#define DEG_TO_RAD(x) (x) * PI / 180.0

namespace abag_parameter
{
    // How many dimension ABAG controller is supposed to control
    const int DIMENSIONS(6);
    const bool USE_ERROR_MAGNITUDE(false);

    const Eigen::VectorXd ERROR_ALPHA = (Eigen::VectorXd(DIMENSIONS) \
                                         << 0.75, 0.75, 0.75, 
                                            0.75, 0.75, 0.75).finished();

    const Eigen::VectorXd BIAS_THRESHOLD = (Eigen::VectorXd(DIMENSIONS) \
                                            << 0.75, 0.75, 0.75, 
                                               0.75, 0.75, 0.75).finished();

    const Eigen::VectorXd BIAS_STEP = (Eigen::VectorXd(DIMENSIONS) \
                                       << 0.000976, 0.000976, 0.000976, 
                                          0.000976, 0.000976, 0.000976).finished();

    const Eigen::VectorXd GAIN_THRESHOLD = (Eigen::VectorXd(DIMENSIONS) \
                                            << 0.5, 0.5, 0.5, 
                                               0.5, 0.5, 0.5).finished();

    const Eigen::VectorXd GAIN_STEP = (Eigen::VectorXd(DIMENSIONS) \
                                       << 0.001953, 0.001953, 0.001953, 
                                          0.001953, 0.001953, 0.001953).finished();

    const Eigen::VectorXd MIN_SAT_LIMIT = (Eigen::VectorXd(DIMENSIONS) \
                                           << 0.0, 0.0, 0.0, 
                                              0.0, 0.0, 0.0).finished();

    const Eigen::VectorXd MAX_SAT_LIMIT = (Eigen::VectorXd(DIMENSIONS) \
                                           << 1.0, 1.0, 1.0, 
                                              1.0, 1.0, 1.0).finished();
}

namespace dynamics_parameter
{
    // Number of task constraints imposed on the robot, i.e. Cartesian DOFS
    const int NUMBER_OF_CONSTRAINTS(6);
}

#endif /* CONTROLLER_CONSTANTS_HPP_ */