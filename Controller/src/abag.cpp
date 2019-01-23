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
ABAG::ABAG(const int num_of_dimensions, 
           const bool reverse_error, 
           const bool use_error_magnitude):
    DIMENSIONS_(num_of_dimensions), 
    REVERSE_ERROR_(reverse_error), USE_ERROR_MAGNITUDE_(use_error_magnitude),
    error_sign_(Eigen::VectorXd::Zero(num_of_dimensions)),
    error_magnitude_(Eigen::VectorXd::Zero(num_of_dimensions)), 
    ONES_(Eigen::VectorXd::Ones(num_of_dimensions)), 
    signal(num_of_dimensions), parameter(num_of_dimensions)
{
    assert(("ABAG Controller not initialized properly", DIMENSIONS_ > 0));
}

// Constructor with all predifined set/s of parameters
ABAG::ABAG(const int num_of_dimensions, const bool reverse_error,
           const bool use_error_magnitude, const Eigen::VectorXd error_alpha,
           const Eigen::VectorXd bias_threshold, const Eigen::VectorXd bias_step, 
           const Eigen::VectorXd gain_threshold, const Eigen::VectorXd gain_step,
           const Eigen::VectorXd min_bias_sat_limit, const Eigen::VectorXd max_bias_sat_limit,
           const Eigen::VectorXd min_gain_sat_limit, const Eigen::VectorXd max_gain_sat_limit,
           const Eigen::VectorXd min_command_sat_limit, const Eigen::VectorXd max_command_sat_limit):
    DIMENSIONS_(num_of_dimensions), 
    REVERSE_ERROR_(reverse_error), USE_ERROR_MAGNITUDE_(use_error_magnitude),
    error_sign_(Eigen::VectorXd::Zero(num_of_dimensions)),
    error_magnitude_(Eigen::VectorXd::Zero(num_of_dimensions)), 
    ONES_(Eigen::VectorXd::Ones(num_of_dimensions)), 
    signal(num_of_dimensions), parameter(error_alpha, 
                                         bias_threshold, bias_step, 
                                         gain_threshold, gain_step, 
                                         min_bias_sat_limit, max_bias_sat_limit, 
                                         min_gain_sat_limit, max_gain_sat_limit, 
                                         min_command_sat_limit, max_command_sat_limit)
{
    // Enforce general parameter constraints and those defined in the original publication 
    assert(("ABAG Controller not initialized properly", DIMENSIONS_ > 0));
    assert(DIMENSIONS_ == parameter.ERROR_ALPHA.rows());
    assert(parameter.ERROR_ALPHA.maxCoeff() < 1.0);
    assert(parameter.ERROR_ALPHA.minCoeff() > 0.0);
    assert(DIMENSIONS_ == parameter.BIAS_THRESHOLD.rows());
    assert(parameter.BIAS_THRESHOLD.maxCoeff() < 1.0);
    assert(parameter.BIAS_THRESHOLD.minCoeff() > 0.0);
    assert(DIMENSIONS_ == parameter.BIAS_STEP.rows());
    assert(parameter.BIAS_STEP.maxCoeff() < 1.0);
    assert(parameter.BIAS_STEP.minCoeff() > 0.0);
    assert(DIMENSIONS_ == parameter.GAIN_THRESHOLD.rows());
    assert(parameter.GAIN_THRESHOLD.maxCoeff() < 1.0);
    assert(parameter.GAIN_THRESHOLD.minCoeff() > 0.0);
    assert(DIMENSIONS_ == parameter.GAIN_STEP.rows());
    assert(parameter.GAIN_STEP.maxCoeff() < 1.0);
    assert(parameter.GAIN_STEP.minCoeff() > 0.0);
    assert(DIMENSIONS_ == parameter.MIN_BIAS_SAT_LIMIT.rows());
    assert(DIMENSIONS_ == parameter.MAX_BIAS_SAT_LIMIT.rows());
    assert(DIMENSIONS_ == parameter.MIN_GAIN_SAT_LIMIT.rows());
    assert(DIMENSIONS_ == parameter.MAX_GAIN_SAT_LIMIT.rows());
    assert(DIMENSIONS_ == parameter.MIN_COMMAND_SAT_LIMIT.rows());
    assert(DIMENSIONS_ == parameter.MAX_COMMAND_SAT_LIMIT.rows());
}

// Update state values for all dimensions and return the command singal - public method
Eigen::VectorXd ABAG::update_state(const Eigen::VectorXd &measured, 
                                   const Eigen::VectorXd &desired)
{   
    assert(DIMENSIONS_ == measured.rows());
    assert(DIMENSIONS_ == desired.rows());

    update_error(measured, desired);
    // std::cout << "Error: \n" << signal.error_.transpose() << std::endl;

    update_bias();
    // std::cout << "Bias: \n" << signal.bias_.transpose() << std::endl;

    update_gain();
    // std::cout << "Gain: \n" << signal.gain_.transpose() << std::endl;

    update_command();
    // std::cout << "Command: \n" << signal.command_.transpose() << std::endl;

    return signal.command_;
}


// - private method
void ABAG::update_error(const Eigen::VectorXd &measured, 
                        const Eigen::VectorXd &desired)
{
    /*
    *   Reversing error is important due to possibility of different controller inputs.
    *   If e.g. velocity is given in Hz or m/s, the error should be reversed w.r.t. 
    *   pseudo code error explained in the original publication. 
    *   Else if, e.g. the velocity is given as a period of rotation, 
    *   the error calculation should be the same as in the orginal pseudo code.
    */
    if (REVERSE_ERROR_) error_magnitude_ = desired - measured;
    else                error_magnitude_ = measured - desired;
    
    error_sign_ = (error_magnitude_).cwiseSign();
    
    // Using error magnitude here instead of sign is an experimental feature!
    // Be carefull with setting "USE_ERROR_MAGNITUDE_" flag!
    signal.error_ = parameter.ERROR_ALPHA.cwiseProduct( signal.error_ ) + \
                    (ONES_ - parameter.ERROR_ALPHA).cwiseProduct( USE_ERROR_MAGNITUDE_? error_magnitude_ : error_sign_ );
}

// - private method
void ABAG::update_bias()
{
    signal.bias_ = saturate_bias(signal.bias_ + \
                                 parameter.BIAS_STEP.cwiseProduct( bias_decision_map() ) \
                                );
}

// - private method
void ABAG::update_gain()
{
    signal.gain_ = saturate_gain(signal.gain_ + \
                                 parameter.GAIN_STEP.cwiseProduct( gain_decision_map() ) \
                                );
}

// - private method
void ABAG::update_command()
{
    signal.command_ = saturate_command( signal.bias_ + signal.gain_.cwiseProduct(error_sign_) );
}


// User customizable function
Eigen::VectorXd ABAG::bias_decision_map()
{
    return heaviside(signal.error_.cwiseAbs() - \
                     parameter.BIAS_THRESHOLD \
                    ).cwiseProduct( (signal.error_ - parameter.BIAS_THRESHOLD).cwiseSign() );
}

// User customizable function
Eigen::VectorXd ABAG::gain_decision_map()
{
    return (signal.error_.cwiseAbs() - parameter.GAIN_THRESHOLD).cwiseSign();
}


// Set all state values to 0, for all dimensions - public method
void ABAG::reset_state()
{
    signal.command_.setZero();
    signal.error_.setZero();
    signal.bias_.setZero();
    signal.gain_.setZero();
}

// Set all state values to 0, for specific dimension - public method
void ABAG::reset_state(const int dimension)
{
    assert(("Not valid dimension number", dimension >= 0));
    assert(("Not valid dimension number", dimension <= (DIMENSIONS_ - 1) ));

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
    assert(("Not valid dimension number", dimension <= (DIMENSIONS_ - 1) ));
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
    assert(("Not valid dimension number", dimension <= (DIMENSIONS_ - 1) ));
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
    assert(("Not valid dimension number", dimension <= (DIMENSIONS_ - 1) ));
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
    assert(("Not valid dimension number", dimension <= (DIMENSIONS_ - 1) ));
    return signal.gain_(dimension);
}


/*
    Setters: Useful for online parameter adaptation
*/
// Set filtering factor parameter for all dimensions - public method
void ABAG::set_error_alpha(const Eigen::VectorXd &error_alpha)
{
    assert(("Not valid dimension number", error_alpha.rows() > 0));
    assert(("Not valid dimension number", error_alpha.rows() <= DIMENSIONS_));
    assert(error_alpha.maxCoeff() < 1.0);
    assert(error_alpha.minCoeff() > 0.0);

    parameter.ERROR_ALPHA = error_alpha;
}

// Set filtering factor parameter for specific dimension - public method
void ABAG::set_error_alpha(const double error_alpha, const int dimension)
{
    assert(("Not valid dimension number", dimension >= 0));
    assert(("Not valid dimension number", dimension <= (DIMENSIONS_ - 1) ));
    assert(error_alpha < 1.0);
    assert(error_alpha > 0.0);

    parameter.ERROR_ALPHA(dimension) = error_alpha;
}


// Set bias threshold parameter for all dimensions - public method
void ABAG::set_bias_threshold(const Eigen::VectorXd &bias_threshold)
{
    assert(("Not valid dimension number", bias_threshold.rows() > 0));
    assert(("Not valid dimension number", bias_threshold.rows() <= DIMENSIONS_));
    assert(bias_threshold.maxCoeff() < 1.0);
    assert(bias_threshold.minCoeff() > 0.0);

    parameter.BIAS_THRESHOLD = bias_threshold;
}

// Set bias threshold parameter for specific dimension - public method
void ABAG::set_bias_threshold(double bias_threshold, const int dimension)
{
    assert(("Not valid dimension number", dimension >= 0));
    assert(("Not valid dimension number", dimension <= (DIMENSIONS_ - 1) ));
    assert(bias_threshold < 1.0);
    assert(bias_threshold > 0.0);

    parameter.BIAS_THRESHOLD(dimension) = bias_threshold;
}


// Set bias step parameter for all dimensions - public method
void ABAG::set_bias_step(const Eigen::VectorXd &bias_step)
{
    assert(("Not valid dimension number", bias_step.rows() > 0));
    assert(("Not valid dimension number", bias_step.rows() <= DIMENSIONS_));
    assert(bias_step.maxCoeff() < 1.0);
    assert(bias_step.minCoeff() > 0.0);

    parameter.BIAS_STEP = bias_step;
}

// Set bias step parameter for specific dimension - public method
void ABAG::set_bias_step(double bias_step, const int dimension)
{
    assert(("Not valid dimension number", dimension >= 0));
    assert(("Not valid dimension number", dimension <= (DIMENSIONS_ - 1) ));
    assert(bias_step < 1.0);
    assert(bias_step > 0.0);

    parameter.BIAS_STEP(dimension) = bias_step;
}


// Set gain threshold parameter for all dimensions - public method
void ABAG::set_gain_threshold(const Eigen::VectorXd &gain_threshold)
{
    assert(("Not valid dimension number", gain_threshold.rows() > 0));
    assert(("Not valid dimension number", gain_threshold.rows() <= DIMENSIONS_));
    assert(gain_threshold.maxCoeff() < 1.0);
    assert(gain_threshold.minCoeff() > 0.0);

    parameter.GAIN_THRESHOLD = gain_threshold;
}

// Set gain threshold parameter for specific dimension - public method
void ABAG::set_gain_threshold(double gain_threshold, const int dimension)
{
    assert(("Not valid dimension number", dimension >= 0));
    assert(("Not valid dimension number", dimension <= (DIMENSIONS_ - 1) ));
    assert(gain_threshold < 1.0);
    assert(gain_threshold > 0.0);

    parameter.GAIN_THRESHOLD(dimension) = gain_threshold;
}

// Set gain step parameter for all dimensions - public method
void ABAG::set_gain_step(const Eigen::VectorXd &gain_step)
{
    assert(("Not valid dimension number", gain_step.rows() > 0));
    assert(("Not valid dimension number", gain_step.rows() <= DIMENSIONS_));
    assert(gain_step.maxCoeff() < 1.0);
    assert(gain_step.minCoeff() > 0.0);

    parameter.GAIN_STEP = gain_step;
}

// Set gain step parameter for specific dimension - public method
void ABAG::set_gain_step(double gain_step, const int dimension)
{
    assert(("Not valid dimension number", dimension >= 0));
    assert(("Not valid dimension number", dimension <= (DIMENSIONS_ - 1) ));
    assert(gain_step < 1.0);
    assert(gain_step > 0.0);

    parameter.GAIN_STEP(dimension) = gain_step;
}

/*
    Help functions
*/
Eigen::VectorXd ABAG::saturate_bias(const Eigen::VectorXd &value)
{   
    assert(parameter.MAX_BIAS_SAT_LIMIT.rows() == value.rows());
    assert(parameter.MIN_BIAS_SAT_LIMIT.rows() == value.rows());
    return value.cwiseMin(parameter.MAX_BIAS_SAT_LIMIT).cwiseMax(parameter.MIN_BIAS_SAT_LIMIT);
}

Eigen::VectorXd ABAG::saturate_gain(const Eigen::VectorXd &value)
{   
    assert(parameter.MAX_GAIN_SAT_LIMIT.rows() == value.rows());
    assert(parameter.MIN_GAIN_SAT_LIMIT.rows() == value.rows());
    return value.cwiseMin(parameter.MAX_GAIN_SAT_LIMIT).cwiseMax(parameter.MIN_GAIN_SAT_LIMIT);
}

Eigen::VectorXd ABAG::saturate_command(const Eigen::VectorXd &value)
{   
    assert(parameter.MAX_COMMAND_SAT_LIMIT.rows() == value.rows());
    assert(parameter.MIN_COMMAND_SAT_LIMIT.rows() == value.rows());
    return value.cwiseMin(parameter.MAX_COMMAND_SAT_LIMIT).cwiseMax(parameter.MIN_COMMAND_SAT_LIMIT);
}

Eigen::VectorXd ABAG::saturate(const Eigen::VectorXd &value, 
                               const Eigen::VectorXd &MIN_LIMIT, 
                               const Eigen::VectorXd &MAX_LIMIT)
{   
    assert(MAX_LIMIT.rows() == value.rows());
    assert(MIN_LIMIT.rows() == value.rows());
    return value.cwiseMin(MAX_LIMIT).cwiseMax(MIN_LIMIT);
}

Eigen::VectorXd ABAG::heaviside(const Eigen::VectorXd &value)
{   
    return 0.5 * (value.cwiseSign() + ONES_);
}