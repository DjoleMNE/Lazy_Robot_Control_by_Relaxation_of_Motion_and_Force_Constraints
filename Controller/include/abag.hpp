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
    ~ABAG(){};

    Eigen::VectorXd get_command();
    double get_command(const int dimension);

    Eigen::VectorXd get_bias();
    double get_bias(const int dimension);

    Eigen::VectorXd get_gain();
    double get_gain(const int dimension);

    void reset_state();
    void reset_state(const int dimension);

  private:
    const int DIMENSIONS_;

    struct abag_signal 
    {
        abag_signal(const int num_of_dimensions):
            command_(num_of_dimensions),
            bias_(num_of_dimensions),
            gain_(num_of_dimensions),
            error_(num_of_dimensions) {
        }
        ~abag_signal(){};
        
        Eigen::VectorXd command_;
        Eigen::VectorXd bias_;
        Eigen::VectorXd gain_;
        Eigen::VectorXd error_;
    } signal;

    struct abag_gain
    {
        abag_gain(const int num_of_dimensions):
            low_pass_(num_of_dimensions, num_of_dimensions),
            bias_(num_of_dimensions, num_of_dimensions),
            gain_(num_of_dimensions, num_of_dimensions) {
        }
        ~abag_gain(){};

        Eigen::MatrixXd low_pass_;
        Eigen::MatrixXd bias_;
        Eigen::MatrixXd gain_;
    } gain;

    std::chrono::steady_clock::time_point loop_start_time_;
    std::chrono::steady_clock::time_point loop_end_time_;
    std::chrono::duration <double, std::micro> loop_interval_{};

    void compute_commands();
    void compute_bias();
    void compute_gain();
};
#endif /* ABAG_HPP_*/