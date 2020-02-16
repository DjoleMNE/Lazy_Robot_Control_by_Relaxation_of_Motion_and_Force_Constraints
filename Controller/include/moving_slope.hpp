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

#ifndef MOVING_SLOPE_HPP_
#define MOVING_SLOPE_HPP_
#include <Eigen/Core>
#include <iostream>
#include <list>
#include <vector>
#include <stdlib.h>     /* abs */

class moving_slope
{
  public:
    moving_slope(const int window_size, const int num_dimensions);
    ~moving_slope(){};

    Eigen::VectorXd update(const Eigen::VectorXd &state);
    double update(const int dimension, const double state);

    void clear();
    void clear(const int dimension);

    Eigen::VectorXd get_slope();
    double get_slope(const int dimension);

  private:
    const int DIMENSIONS_, WINDOW_SIZE_;
    Eigen::VectorXd slopes_;
    std::vector<int> indexes_;
    std::vector<std::list<double>> windows_;
};
#endif /* MOVING_SLOPE_HPP_*/