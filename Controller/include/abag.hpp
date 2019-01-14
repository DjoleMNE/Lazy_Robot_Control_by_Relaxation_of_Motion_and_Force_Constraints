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

#ifndef ABAG_HPP_
#define ABAG_HPP_
#include <Eigen/Core>
#include <iostream>
#include <sstream>
#include <fstream>
#include <chrono>
#include <time.h>
#include <thread> 
#include <cmath>
#include <stdlib.h>     /* abs */

class ABAG
{
  public:
    ABAG(const int num_of_dimensions);
    ABAG(const int num_of_dimensions, const Eigen::VectorXd alpha,
         const Eigen::VectorXd bias_threshold, const Eigen::VectorXd bias_step, 
         const Eigen::VectorXd gain_threshold, const Eigen::VectorXd gain_step,
         const Eigen::VectorXd min_sat_limit, const Eigen::VectorXd max_sat_limit);
    ~ABAG(){};

    Eigen::VectorXd update_state(const Eigen::VectorXd measured, 
                                 const Eigen::VectorXd desired);
    Eigen::VectorXd get_command();
    double get_command(const int dimension);

    Eigen::VectorXd get_error();
    double get_error(const int dimension);

    Eigen::VectorXd get_bias();
    double get_bias(const int dimension);

    Eigen::VectorXd get_gain();
    double get_gain(const int dimension);

    void set_alpha(const Eigen::VectorXd alpha);
    void set_alpha(const double alpha , const int dimension);
    
    void set_bias_threshold(const Eigen::VectorXd bias_step);
    void set_bias_threshold(double bias_step, const int dimension);

    void set_bias_step(const Eigen::VectorXd bias_step);
    void set_bias_step(double bias_step, const int dimension);

    void set_gain_threshold(const Eigen::VectorXd gain_step);
    void set_gain_threshold(double gain_step, const int dimension);

    void set_gain_step(const Eigen::VectorXd gain_step);
    void set_gain_step(double gain_step, const int dimension);

    void reset_state();
    void reset_state(const int dimension);

  private:
    const int DIMENSIONS_;
    const Eigen::VectorXd ones_;
    Eigen::VectorXd error_sign_;
    Eigen::VectorXd error_magnitude_;

    struct abag_signal 
    {
        abag_signal(const int num_of_dimensions) 
        {
            command_ = Eigen::VectorXd::Zero(num_of_dimensions);
            error_ = Eigen::VectorXd::Zero(num_of_dimensions);
            bias_ = Eigen::VectorXd::Zero(num_of_dimensions);
            gain_ = Eigen::VectorXd::Zero(num_of_dimensions);
        }
        ~abag_signal(){};

        Eigen::VectorXd command_;
        Eigen::VectorXd error_;
        Eigen::VectorXd bias_;
        Eigen::VectorXd gain_;
    } signal;

    struct abag_parameter
    {
        abag_parameter(const int num_of_dimensions): 
            ERROR_ALPHA(Eigen::VectorXd::Zero(num_of_dimensions)),
            BIAS_THRESHOLD(Eigen::VectorXd::Zero(num_of_dimensions)),
            BIAS_STEP(Eigen::VectorXd::Zero(num_of_dimensions)),
            GAIN_THRESHOLD(Eigen::VectorXd::Zero(num_of_dimensions)),
            GAIN_STEP(Eigen::VectorXd::Zero(num_of_dimensions)),
            MIN_SAT_LIMIT(Eigen::VectorXd::Zero(num_of_dimensions)),
            MAX_SAT_LIMIT(Eigen::VectorXd::Zero(num_of_dimensions)) {};

        abag_parameter(const Eigen::VectorXd alpha, 
                       const Eigen::VectorXd bias_threshold, 
                       const Eigen::VectorXd bias_step,
                       const Eigen::VectorXd gain_threshold, 
                       const Eigen::VectorXd gain_step,
                       const Eigen::VectorXd min_sat_limit, 
                       const Eigen::VectorXd max_sat_limit):
            ERROR_ALPHA(alpha),
            BIAS_THRESHOLD(bias_threshold),
            BIAS_STEP(bias_step),
            GAIN_THRESHOLD(gain_threshold),
            GAIN_STEP(gain_step),
            MIN_SAT_LIMIT(min_sat_limit),
            MAX_SAT_LIMIT(max_sat_limit){};

        ~abag_parameter(){};

        Eigen::VectorXd ERROR_ALPHA;
        Eigen::VectorXd BIAS_THRESHOLD;
        Eigen::VectorXd BIAS_STEP;
        Eigen::VectorXd GAIN_THRESHOLD;
        Eigen::VectorXd GAIN_STEP;
        Eigen::VectorXd MIN_SAT_LIMIT;
        Eigen::VectorXd MAX_SAT_LIMIT;
    } parameter;

    void update_error();
    void update_bias();
    void update_gain();

    // User customizable functions
    Eigen::VectorXd bias_decision_map();
    Eigen::VectorXd gain_decision_map();

    // Help functions
    Eigen::VectorXd saturate(const Eigen::VectorXd value);
    Eigen::VectorXd heaviside(const Eigen::VectorXd value);
};
#endif /* ABAG_HPP_*/