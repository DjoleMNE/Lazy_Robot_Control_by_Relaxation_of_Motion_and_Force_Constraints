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
#include <stdlib.h>     /* abs */
#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
#include <unistd.h>
#include <cmath>
#include <assert.h>
#include <deque>

enum m_profile 
{
    CONSTANT = 0,
    TANH = 1,
    S_CURVE = 2,
    STEP = 3
};

namespace motion_profile
{
    std::deque<double> ramp_array(const double start, const double stop,
                                  const double rate, const double threshold);
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

    // Path generators for X-Z plane
    void draw_sine_xz(std::vector< std::vector<double> > &path_points,
                      const double frequency_start,
                      const double frequency_end,
                      const double amplitude,
                      const double x_scale, const double offset_x, 
                      const double offset_y, const double offset_z);
    void draw_inf_sign_xz(std::vector< std::vector<double> > &path_points,
                          const double length, const double height, 
                          const double amplitude,
                          const double x_scale, const double offset_x, 
                          const double offset_y, const double offset_z);
    void draw_step_xz(std::vector< std::vector<double> > &path_points,
                      const int step_size,
                      const double x_scale, const double offset_x, 
                      const double offset_y, const double offset_z);

    // Path generators for X-Y plane
    void draw_sine_xy(std::vector< std::vector<double> > &path_points,
                      const double frequency_start,
                      const double frequency_end,
                      const double amplitude,
                      const double x_scale, const double offset_x, 
                      const double offset_y, const double offset_z);
    void draw_inf_sign_xy(std::vector< std::vector<double> > &path_points,
                          const double length, const double height, 
                          const double amplitude,
                          const double x_scale, const double offset_x, 
                          const double offset_y, const double offset_z);
    void draw_step_xy(std::vector< std::vector<double> > &path_points,
                      const int step_size,
                      const double x_scale, const double offset_x, 
                      const double offset_y, const double offset_z);
};
#endif /* MOTION_PROFILE_HPP */