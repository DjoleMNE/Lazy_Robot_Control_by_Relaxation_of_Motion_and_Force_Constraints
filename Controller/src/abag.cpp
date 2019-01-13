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
ABAG::ABAG(const int num_of_dimensions, const Eigen::VectorXd low_pass, 
           const Eigen::VectorXd bias, const Eigen::VectorXd gain):
    DIMENSIONS_(num_of_dimensions),
    signal(num_of_dimensions), 
    parameter(low_pass, bias, gain)
{
    assert(("ABAG Controller not initialized properly", DIMENSIONS_ > 0));
    assert(DIMENSIONS_ == parameter.low_pass_.rows());
    assert(DIMENSIONS_ == parameter.bias_.rows());
    assert(DIMENSIONS_ == parameter.gain_.rows());
}

// - private method
void ABAG::compute_commands()
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

// Set all state values to 0 - public method
void ABAG::reset_state()
{
    //TODO
}

// Set all state values to 0 - public method
void ABAG::reset_state(const int dimension)
{
    //TODO
}

/*
    Getters
*/
// Get command values for all dimensions - public method
Eigen::VectorXd ABAG::get_command()
{
    return signal.command_;
}

// Get command value for specific dimension - public method
double ABAG::get_command(const int dimension)
{
    return signal.command_(dimension);
}

// Get bias values for all dimensions - public method
Eigen::VectorXd ABAG::get_bias()
{
    return signal.bias_;
}

// Get bias value for specific dimension - public method
double ABAG::get_bias(const int dimension)
{
    return signal.bias_(dimension);
}

// Get gain values for all dimensions - public method
Eigen::VectorXd ABAG::get_gain()
{
    return signal.gain_;
}

// Get gain value for specific dimension - public method
double ABAG::get_gain(const int dimension)
{
    return signal.gain_(dimension);
}

/*
    Setters
*/
// Set bias parameter for all dimensions - public method
void ABAG::set_low_pass_parameter(const Eigen::VectorXd low_pass)
{
    parameter.low_pass_ = low_pass;
}

// Set bias parameter for specific dimension - public method
void ABAG::set_low_pass_parameter(const double low_pass, const int dimension)
{
    parameter.low_pass_(dimension) = low_pass;
}

// Set bias parameter for all dimensions - public method
void ABAG::set_bias_parameter(const Eigen::VectorXd bias)
{
    parameter.bias_ = bias;
}

// Set bias parameter for specific dimension - public method
void ABAG::set_bias_parameter(double bias, const int dimension)
{
    parameter.bias_(dimension) = bias;
}

// Set gain parameter for all dimensions - public method
void ABAG::set_gain_parameter(const Eigen::VectorXd gain)
{
    parameter.gain_ = gain;
}

// Set gain parameter for specific dimension - public method
void ABAG::set_gain_parameter(double gain, const int dimension)
{
    parameter.gain_(dimension) = gain;
}
