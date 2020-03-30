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

#include <motion_profile.hpp>

namespace motion_profile
{

    std::deque<double> ramp_array(const double start_state, const double end_state,
                                  const double step, const double threshold)
    {
        std::deque<double> setpoint_array;
        double setpoint = start_state + step;

        while ((step > 0.0)? setpoint < end_state : setpoint > end_state)
        {
            if (std::fabs(setpoint) < std::fabs(threshold)) break;

            setpoint_array.push_back(setpoint);
            setpoint = setpoint_array.back() + step;
        }

        setpoint_array.push_back(end_state);
        return setpoint_array;
    }


    double tanh_function(const double state,
                         const double offset,
                         const double amplitude,
                         const double slope)
    {
        // Two-parameter-scaled tanh function
        return offset + amplitude * std::tanh(slope * state);
    }

    double tanh_inverse_function(const double state,
                                 const double offset,
                                 const double amplitude,
                                 const double slope)
    {
        // Three-parameter-scaled tanh function
        if(state < 0.001) return offset + amplitude * std::tanh(slope / 0.001);
        else return offset + amplitude * std::tanh(slope / state);
    }

    double step_function(const double state,
                         const double magnitude,
                         const double delta_slope,
                         const double upper_limit,
                         const double lower_limit)
    {
        if(state >= upper_limit) return magnitude;
        else if (state < upper_limit && state > lower_limit) return (1.0 + delta_slope) * magnitude;
        else return (1.0 + 2 * delta_slope) * magnitude;
    }

    double negative_step_function(const double state,
                                  const double magnitude,
                                  const double delta_slope,
                                  const double upper_limit,
                                  const double lower_limit)
    {
        if(state >= upper_limit) return magnitude;
        else if (state < upper_limit && state > lower_limit) return (1.0 - delta_slope) * magnitude;
        else return (1.0 - 2 * delta_slope) * magnitude;
    }


    double s_curve_function(const double state,
                            const double offset,
                            const double amplitude,
                            const double slope)
    {
        // if(state > 0.61) return offset + amplitude * std::sin(slope * 0.61);
        // else return offset + amplitude * std::sin(slope * state);
        return offset + amplitude * std::sin(slope * state);
    }


    void draw_sine_xz(std::vector< std::vector<double> > &path_points,
                      const double frequency_start,
                      const double frequency_end, 
                      const double amplitude,
                      const double x_scale, const double offset_x, 
                      const double offset_y, const double offset_z)
    {
        double x_y, z;
        double frequency = 0.0;
        double freq_step = 0.0;
        if (frequency_end > frequency_start) freq_step = (frequency_end - frequency_start) / (static_cast<float>(path_points.size() - 1));

        for(int i = 0; (unsigned)i < path_points.size(); i++)
        {
            // the angle is plotted on the x-plane
            x_y = i;
            frequency = frequency_start + freq_step * i;

            // the sine function is plotted along the z-axis
            z = amplitude * std::sin(2 * M_PI * frequency * (x_y / path_points.size()));

            path_points[i][0] = x_y * x_scale + offset_x;
            path_points[i][1] = 0.0           + offset_y;
            path_points[i][2] = z             + offset_z;
        }
    }

    void draw_inf_sign_xz(std::vector< std::vector<double> > &path_points,
                          const double length, const double height, 
                          const double amplitude,
                          const double x_scale, const double offset_x, 
                          const double offset_y, const double offset_z)
    {
        double x_y, z;

        for(int i = 0; (unsigned)i < path_points.size(); i++)
        {
            // the angle is plotted on the x-y-plane
            x_y = x_scale * amplitude * (-length / 2.0) * std::cos(2.0 * M_PI * i / path_points.size()) + length / 2.0;

            // the sine function is plotted along the z-axis
            z = amplitude * (height / 2) * std::sin(4 * M_PI * i / path_points.size());

            path_points[i][0] = x_y + offset_x;
            path_points[i][1] = 0.0 + offset_y;
            path_points[i][2] = z   + offset_z;
        }
    }

    void draw_step_xz(std::vector< std::vector<double> > &path_points,
                      const int step_size,
                      const double x_scale, const double offset_x, 
                      const double offset_y, const double offset_z)
    {
        double x_y, z;
        int offset = int(path_points.size() / step_size);

        for(int i = 0; (unsigned)i < path_points.size(); i++)
        {
            x_y = x_scale * i;

            if      (i < offset)                       z =  0.0;
            else if (i >     offset && i < 2 * offset) z =  0.07;
            else if (i > 2 * offset && i < 3 * offset) z = -0.05;
            else if (i > 3 * offset && i < 4 * offset) z =  0.04;
            else                                       z =  0.01;

            path_points[i][0] = x_y + offset_x;
            path_points[i][1] = 0.0 + offset_y;
            path_points[i][2] = z   + offset_z;
        }
    }



    void draw_sine_xy(std::vector< std::vector<double> > &path_points,
                      const double frequency_start,
                      const double frequency_end, 
                      const double amplitude,
                      const double x_scale, const double offset_x, 
                      const double offset_y, const double offset_z)
    {
        double x, y;
        double frequency = 0.0;
        double freq_step = 0.0;
        if (frequency_end > frequency_start) freq_step = (frequency_end - frequency_start) / (static_cast<float>(path_points.size() - 1));

        for(int i = 0; (unsigned)i < path_points.size(); i++)
        {
            // the angle is plotted on the x-plane
            x = i;
            frequency = frequency_start + freq_step * i;

            // the sine function is plotted along the z-axis
            y = amplitude * std::sin(2 * M_PI * frequency * (x / path_points.size()));

            path_points[i][0] = x * x_scale + offset_x;
            path_points[i][1] = y           + offset_y;
            path_points[i][2] = 0.0         + offset_z;
        }
    }

    void draw_inf_sign_xy(std::vector< std::vector<double> > &path_points,
                          const double length, const double height, 
                          const double amplitude,
                          const double x_scale, const double offset_x, 
                          const double offset_y, const double offset_z)
    {
        double x, y;

        for(int i = 0; (unsigned)i < path_points.size(); i++)
        {
            // the angle is plotted on the x-y-plane
            x = x_scale * amplitude * (-length / 2.0) * std::cos(2.0 * M_PI * i / path_points.size()) + length / 2.0;

            // the sine function is plotted along the y-axis
            y = amplitude * (height / 2) * std::sin(4 * M_PI * i / path_points.size());

            path_points[i][0] = x    + offset_x;
            path_points[i][1] = y    + offset_y;
            path_points[i][2] = 0.0  + offset_z;
        }
    }

    void draw_step_xy(std::vector< std::vector<double> > &path_points,
                      const int step_size,
                      const double x_scale, const double offset_x, 
                      const double offset_y, const double offset_z)
    {
        double x, y;
        int offset = int(path_points.size() / step_size);

        for (int i = 0; (unsigned)i < path_points.size(); i++)
        {
            x = x_scale * i;

            if      (i <=     offset                  ) y =  0.0;
            else if (i >     offset && i <= 2 * offset) y =  0.02;
            else if (i > 2 * offset && i <= 3 * offset) y = -0.03;
            else if (i > 3 * offset && i <= 4 * offset) y =  0.01;
            else                                        y =  0.04;

            path_points[i][0] = x   + offset_x;
            path_points[i][1] = y   + offset_y;
            path_points[i][2] = 0.0 + offset_z;
        }
    }
}