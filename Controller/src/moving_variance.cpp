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

#include <moving_variance.hpp>

// Constructor without the predefined set/s of parameters
moving_variance::moving_variance(const int window_size, const int num_dimensions):
    WINDOW_SIZE_(window_size), DIMENSIONS_(num_dimensions),
    indexes_(num_dimensions, 0),
    means_(Eigen::VectorXd::Zero(num_dimensions)),
    variances_(Eigen::VectorXd::Zero(num_dimensions)),
    windows_(num_dimensions, std::list<double>())
{
    assert(("Moving Variance algorithm not initialized properly", DIMENSIONS_ > 0));
    clear();
}

Eigen::VectorXd moving_variance::update(const Eigen::VectorXd &state)
{   
    assert(DIMENSIONS_ == state.rows());
    Eigen::VectorXd variance(DIMENSIONS_);

    for (int i = 0; i < DIMENSIONS_; i++)
        variance(i) = update(i, state(i));

    return variance;
}

double moving_variance::update(const int dimension, const double state)
{
    assert(dimension >= 0);
    assert(dimension < DIMENSIONS_);

    windows_[dimension].push_back(state);

    if (indexes_[dimension] < WINDOW_SIZE_)
    { // Calculating variance for the initial window elements
        indexes_[dimension]++;
        double delta           = state - means_(dimension);
        means_(dimension)     += delta / static_cast<float>(indexes_[dimension]);
        variances_(dimension) += delta * (state - means_(dimension));
    }
    else
    { // Calculating variance for all other window elements
        double removed_state   = windows_[dimension].front(); windows_[dimension].pop_front();
        double old_mean        = means_(dimension);
        means_(dimension)     += (state - removed_state) / static_cast<float>(WINDOW_SIZE_);
        variances_(dimension) += (state + removed_state - old_mean - means_(dimension)) * (state - removed_state);
    }

    // Normalize the value
    return variances_(dimension) / static_cast<float>(windows_[dimension].size());
}

void moving_variance::clear()
{
    for (int i = 0; i < DIMENSIONS_; i++)
        clear(i);
}

void moving_variance::clear(const int dimension)
{
    assert(dimension >= 0);
    assert(dimension < DIMENSIONS_);

    means_(dimension)     = 0.0;
    indexes_[dimension]   = 0;
    variances_(dimension) = 0.0;
    windows_[dimension].clear();
}

Eigen::VectorXd moving_variance::get_variance()
{
    Eigen::VectorXd variance(DIMENSIONS_);
    for (int i = 0; i < DIMENSIONS_; i++)
        variance(i) = get_variance(i);
    return variance;
}

double moving_variance::get_variance(const int dimension)
{
    assert(dimension >= 0);
    assert(dimension < DIMENSIONS_);

    // Normalize the value
    if (windows_[dimension].size() == 0) return 0.0;
    return variances_(dimension) / static_cast<float>(windows_[dimension].size());
}

Eigen::VectorXd moving_variance::get_mean()
{
    return means_;
}

double moving_variance::get_mean(const int dimension)
{
    assert(dimension >= 0);
    assert(dimension < DIMENSIONS_);

    return means_(dimension);
}