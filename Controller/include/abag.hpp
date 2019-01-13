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
#include <state_specification.hpp>
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

    int get_command();
    int get_command(int dimension);
    int get_bias();
    int get_bias(int dimension);
    int get_gain();
    int get_gain(int dimension);

    void reset_state();
    void reset_state(int dimension);

  private:
    const int DIMENSIONS_;

    std::chrono::steady_clock::time_point loop_start_time_;
    std::chrono::steady_clock::time_point loop_end_time_;
    std::chrono::duration <double, std::micro> loop_interval_{};

    int compute_commands();
    int compute_bias();
    int compute_gain();
};
#endif /* ABAG_HPP_*/