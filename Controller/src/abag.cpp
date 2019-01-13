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

#include <abag.hpp>

// Constructor without the predifined set/s of parameters
ABAG::ABAG(const int num_of_dimensions):
    DIMENSIONS_(num_of_dimensions),
    signal(num_of_dimensions), 
    parameter(num_of_dimensions)
{
    assert(("ABAG Controller not initialized properly", DIMENSIONS_ > 0));
}

// Constructor with the predifined set/s of parameters
ABAG::ABAG(const int num_of_dimensions, const Eigen::VectorXd filtering_factor,
           const Eigen::VectorXd bias_threshold, const Eigen::VectorXd bias_step, 
           const Eigen::VectorXd gain_threshold, const Eigen::VectorXd gain_step):
    DIMENSIONS_(num_of_dimensions),
    signal(num_of_dimensions), 
    parameter(filtering_factor, bias_threshold, 
              bias_step, gain_threshold, gain_step)
{
    assert(("ABAG Controller not initialized properly", DIMENSIONS_ > 0));
    assert(DIMENSIONS_ == parameter.filtering_factor_.rows());
    assert(DIMENSIONS_ == parameter.bias_threshold_.rows());
    assert(DIMENSIONS_ == parameter.bias_step_.rows());
    assert(DIMENSIONS_ == parameter.gain_threshold_.rows());
    assert(DIMENSIONS_ == parameter.gain_step_.rows());
}


// - private method
void ABAG::compute_error()
{
    //TODO
}

// - private method
void ABAG::compute_bias()
{
    //TODO
}

// - private method
void ABAG::compute_gain()
{
    //TODO
}

// - private method
void ABAG::compute_commands()
{
    //TODO
} 

// Set all state values to 0 - public method
void ABAG::reset_state()
{
    signal.command_.setZero();
    signal.error_.setZero();
    signal.bias_.setZero();
    signal.gain_.setZero();
}

// Set all state values to 0 - public method
void ABAG::reset_state(const int dimension)
{
    assert(("Not valid dimension number", dimension >= 0));
    assert(("Not valid dimension number", dimension <= (DIMENSIONS_ - 1)));

    signal.command_(dimension) = 0;
    signal.error_(dimension) = 0;
    signal.bias_(dimension) = 0;
    signal.gain_(dimension) = 0;
}

/*
    Getters
*/
// Get command signal values for all dimensions - public method
Eigen::VectorXd ABAG::get_command()
{
    return signal.command_;
}

// Get command signal value for specific dimension - public method
double ABAG::get_command(const int dimension)
{
    assert(("Not valid dimension number", dimension >= 0));
    assert(("Not valid dimension number", dimension <= (DIMENSIONS_ - 1)));
    // compute_commands()
    return signal.command_(dimension);
}

// Get error signal values for all dimensions - public method
Eigen::VectorXd ABAG::get_error()
{
    return signal.error_;
}

// Get error signal value for specific dimension - public method
double ABAG::get_error(const int dimension)
{
    assert(("Not valid dimension number", dimension >= 0));
    assert(("Not valid dimension number", dimension <= (DIMENSIONS_ - 1)));
    return signal.error_(dimension);
}

// Get bias signal values for all dimensions - public method
Eigen::VectorXd ABAG::get_bias()
{
    return signal.bias_;
}

// Get bias signal value for specific dimension - public method
double ABAG::get_bias(const int dimension)
{
    assert(("Not valid dimension number", dimension >= 0));
    assert(("Not valid dimension number", dimension <= (DIMENSIONS_ - 1)));
    return signal.bias_(dimension);
}

// Get gain signal values for all dimensions - public method
Eigen::VectorXd ABAG::get_gain()
{
    return signal.gain_;
}

// Get gain signal value for specific dimension - public method
double ABAG::get_gain(const int dimension)
{
    assert(("Not valid dimension number", dimension >= 0));
    assert(("Not valid dimension number", dimension <= (DIMENSIONS_ - 1)));
    return signal.gain_(dimension);
}

/*
    Setters
*/
// Set filtering factor parameter for all dimensions - public method
void ABAG::set_filtering_factor(const Eigen::VectorXd filtering_factor)
{
    assert(("Not valid dimension number", filtering_factor.rows() > 0));
    assert(("Not valid dimension number", filtering_factor.rows() <= DIMENSIONS_));

    parameter.filtering_factor_ = filtering_factor;
}

// Set filtering factor parameter for specific dimension - public method
void ABAG::set_filtering_factor(const double filtering_factor, const int dimension)
{
    assert(("Not valid dimension number", dimension >= 0));
    assert(("Not valid dimension number", dimension <= (DIMENSIONS_ - 1)));
    
    parameter.filtering_factor_(dimension) = filtering_factor;
}

// Set bias threshold parameter for all dimensions - public method
void ABAG::set_bias_threshold(const Eigen::VectorXd bias_threshold)
{
    assert(("Not valid dimension number", bias_threshold.rows() > 0));
    assert(("Not valid dimension number", bias_threshold.rows() <= DIMENSIONS_));

    parameter.bias_threshold_ = bias_threshold;
}

// Set bias threshold parameter for specific dimension - public method
void ABAG::set_bias_threshold(double bias_threshold, const int dimension)
{
    assert(("Not valid dimension number", dimension >= 0));
    assert(("Not valid dimension number", dimension <= (DIMENSIONS_ - 1)));

    parameter.bias_threshold_(dimension) = bias_threshold;
}

// Set bias step parameter for all dimensions - public method
void ABAG::set_bias_step(const Eigen::VectorXd bias_step)
{
    assert(("Not valid dimension number", bias_step.rows() > 0));
    assert(("Not valid dimension number", bias_step.rows() <= DIMENSIONS_));

    parameter.bias_step_ = bias_step;
}

// Set bias step parameter for specific dimension - public method
void ABAG::set_bias_step(double bias_step, const int dimension)
{
    assert(("Not valid dimension number", dimension >= 0));
    assert(("Not valid dimension number", dimension <= (DIMENSIONS_ - 1)));

    parameter.bias_step_(dimension) = bias_step;
}

// Set gain threshold parameter for all dimensions - public method
void ABAG::set_gain_threshold(const Eigen::VectorXd gain_threshold)
{
    assert(("Not valid dimension number", gain_threshold.rows() > 0));
    assert(("Not valid dimension number", gain_threshold.rows() <= DIMENSIONS_));

    parameter.gain_threshold_ = gain_threshold;
}

// Set gain threshold parameter for specific dimension - public method
void ABAG::set_gain_threshold(double gain_threshold, const int dimension)
{
    assert(("Not valid dimension number", dimension >= 0));
    assert(("Not valid dimension number", dimension <= (DIMENSIONS_ - 1)));

    parameter.gain_threshold_(dimension) = gain_threshold;
}

// Set gain step parameter for all dimensions - public method
void ABAG::set_gain_step(const Eigen::VectorXd gain_step)
{
    assert(("Not valid dimension number", gain_step.rows() > 0));
    assert(("Not valid dimension number", gain_step.rows() <= DIMENSIONS_));

    parameter.gain_step_ = gain_step;
}

// Set gain step parameter for specific dimension - public method
void ABAG::set_gain_step(double gain_step, const int dimension)
{
    assert(("Not valid dimension number", dimension >= 0));
    assert(("Not valid dimension number", dimension <= (DIMENSIONS_ - 1)));

    parameter.gain_step_(dimension) = gain_step;
}
