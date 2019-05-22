/*
Author(s): Djordje Vukcevic, Sven Schneider
Institute: Hochschule Bonn-Rhein-Sieg

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

#ifndef MOTION_PROFILE_HPP
#define MOTION_PROFILE_HPP
// #include <state_specification.hpp>
// #include <constants.hpp>
// #include <fk_vereshchagin.hpp>
// #include <kdl_eigen_conversions.hpp>
#include <iostream>
#include <sstream>
#include <fstream>
#include <unistd.h>
#include <cmath>
#include <stdlib.h>     /* abs */

enum m_profile 
{
    CONSTANT = -1,
    S_CURVE = 0,
    TANH = 1
};

namespace motion_profile
{
        double tanh_function(const double state,
                             const double offset,
                             const double amplitude,
                             const double slope);
        double tanh_inverse_function(const double state,
                                     const double offset,
                                     const double amplitude,
                                     const double slope);
        double step_function(const double state,
                             const double magnitude,
                             const double delta_slope,
                             const double upper_limit,
                             const double lower_limit);
        double negative_step_function(const double state,
                                      const double magnitude,
                                      const double delta_slope,
                                      const double upper_limit,
                                      const double lower_limit);
        double s_curve_function(const double state,
                                const double offset,
                                const double amplitude,
                                const double slope);
};
#endif /* MOTION_PROFILE_HPP */