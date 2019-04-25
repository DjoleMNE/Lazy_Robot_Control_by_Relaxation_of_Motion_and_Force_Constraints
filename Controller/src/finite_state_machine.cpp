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

#include <finite_state_machine.hpp>

finite_state_machine::finite_state_machine()
{
}

double finite_state_machine::tanh_decision_map(const double state,
                                               const double amplitude,
                                               const double slope)
{
   // Two-parameter-scaled tanh function
   return amplitude * std::tanh(slope * state);
}

double finite_state_machine::tanh_inverse_decision_map(const double state,
                                                       const double offset,
                                                       const double amplitude,
                                                       const double slope)
{
   // Three-parameter-scaled tanh function
   if(state < 0.001) return offset + amplitude * std::tanh(slope / 0.001);
   else return offset + amplitude * std::tanh(slope / state);
}

double finite_state_machine::step_decision_map(const double state,
                                               const double magnitude,
                                               const double delta_slope,
                                               const double upper_limit,
                                               const double lower_limit)
{
    if(state >= upper_limit) return magnitude;
    else if (state < upper_limit && state > lower_limit) return (1.0 + delta_slope) * magnitude;
    else return (1.0 + 2 * delta_slope) * magnitude;
}

double finite_state_machine::negative_step_decision_map(const double state,
                                                        const double magnitude,
                                                        const double delta_slope,
                                                        const double upper_limit,
                                                        const double lower_limit)
{
    if(state >= upper_limit) return magnitude;
    else if (state < upper_limit && state > lower_limit) return (1.0 - delta_slope) * magnitude;
    else return (1.0 - 2 * delta_slope) * magnitude;
}