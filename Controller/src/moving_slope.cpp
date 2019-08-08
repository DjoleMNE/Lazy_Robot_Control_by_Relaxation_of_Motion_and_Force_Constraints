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

#include <moving_slope.hpp>

// Constructor without the predefined set/s of parameters
moving_slope::moving_slope(const int window_size, const int num_dimensions):
    WINDOW_SIZE_(window_size), DIMENSIONS_(num_dimensions),
    slopes_(Eigen::VectorXd::Zero(num_dimensions)),
    indexes_(num_dimensions, 0),
    windows_(num_dimensions, std::list<double>())
{
    assert(("Moving Slope algorithm not initialized properly", DIMENSIONS_ > 0));
    clear();
}

Eigen::VectorXd moving_slope::update(const Eigen::VectorXd &state)
{   
    assert(DIMENSIONS_ == state.rows());

    for (int i = 0; i < DIMENSIONS_; i++)
        update(i, state(i));

    return slopes_;
}

double moving_slope::update(const int dimension, const double state)
{
    assert(dimension >= 0);
    assert(dimension < DIMENSIONS_);

    windows_[dimension].push_back(state);

    if (indexes_[dimension] < WINDOW_SIZE_)
    { // Calculating slope for the initial window elements
        indexes_[dimension]++;
        slopes_(dimension) = 0.002;
    }
    else
    { // Calculating slope for all other window elements
        double removed_state = windows_[dimension].front(); windows_[dimension].pop_front();
        slopes_(dimension)   = (state - removed_state) / static_cast<float>(WINDOW_SIZE_);
    }

    return slopes_(dimension);
}

void moving_slope::clear()
{
    for (int i = 0; i < DIMENSIONS_; i++)
        clear(i);
}

void moving_slope::clear(const int dimension)
{
    assert(dimension >= 0);
    assert(dimension < DIMENSIONS_);

    slopes_(dimension)  = 0.0;
    indexes_[dimension] = 0;
    windows_[dimension].clear();
}

Eigen::VectorXd moving_slope::get_slope()
{
    return slopes_;
}

double moving_slope::get_slope(const int dimension)
{
    assert(dimension >= 0);
    assert(dimension < DIMENSIONS_);

    return slopes_(dimension);
}