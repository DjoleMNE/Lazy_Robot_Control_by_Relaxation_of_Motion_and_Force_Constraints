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
    ABAG(const int num_of_dimensions, const Eigen::VectorXd low_pass, 
         const Eigen::VectorXd bias, const Eigen::VectorXd gain);
    ~ABAG(){};

    Eigen::VectorXd get_command();
    double get_command(const int dimension);

    Eigen::VectorXd get_bias();
    double get_bias(const int dimension);

    Eigen::VectorXd get_gain();
    double get_gain(const int dimension);

    void set_bias_parameter(double bias, const int dimension);
    void set_bias_parameter(const Eigen::VectorXd bias);

    void set_low_pass_parameter(const double low_pass, const int dimension);
    void set_low_pass_parameter(const Eigen::VectorXd low_pass);

    void set_gain_parameter(const Eigen::VectorXd gain);
    void set_gain_parameter(double gain, const int dimension);

    void reset_state();
    void reset_state(const int dimension);

  private:
    const int DIMENSIONS_;

    struct abag_signal 
    {
        abag_signal(const int num_of_dimensions) 
        {
            command_ = Eigen::VectorXd::Zero(num_of_dimensions);
            bias_ = Eigen::VectorXd::Zero(num_of_dimensions);
            gain_ = Eigen::VectorXd::Zero(num_of_dimensions);
            error_ = Eigen::VectorXd::Zero(num_of_dimensions);
        }
        ~abag_signal(){};
        
        Eigen::VectorXd command_;
        Eigen::VectorXd bias_;
        Eigen::VectorXd gain_;
        Eigen::VectorXd error_;
    } signal;

    struct abag_parameter
    {
        abag_parameter(const int num_of_dimensions)
        {
            low_pass_ = Eigen::VectorXd::Zero(num_of_dimensions);
            bias_ = Eigen::VectorXd::Zero(num_of_dimensions);
            gain_ = Eigen::VectorXd::Zero(num_of_dimensions);
        }
        
        abag_parameter(const Eigen::VectorXd low_pass, 
                       const Eigen::VectorXd bias, 
                       const Eigen::VectorXd gain):
            low_pass_(low_pass),
            bias_(bias),
            gain_(gain){};

        ~abag_parameter(){};

        Eigen::VectorXd low_pass_;
        Eigen::VectorXd bias_;
        Eigen::VectorXd gain_;
    } parameter;

    void compute_commands();
    void compute_bias();
    void compute_gain();
};
#endif /* ABAG_HPP_*/