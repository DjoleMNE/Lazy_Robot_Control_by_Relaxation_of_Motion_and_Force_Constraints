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

ABAG::ABAG(const int num_of_dimensions):
    DIMENSIONS_(num_of_dimensions)
{
    assert(("Controller not initialized properly", DIMENSIONS_ == 0));
}

// Set all state values to 0 - public method
void ABAG::reset_state()
{
    //TODO
}

// Set all state values to 0 - public method
void ABAG::reset_state(int dimension)
{
    //TODO
}

// Get command values for all dimensions - public method
int ABAG::get_command()
{
    //TODO
}

// Get command value for specific dimension - public method
int ABAG::get_command(int dimension)
{
    //TODO
}

// Get bias values for all dimensions - public method
int ABAG::get_bias()
{
    //TODO
}

// Get bias value for specific dimension - public method
int ABAG::get_bias(int dimension)
{
    //TODO
}

// Get gain values for all dimensions - public method
int ABAG::get_gain()
{
    //TODO
}

// Get gain value for specific dimension - public method
int ABAG::get_gain(int dimension)
{
    //TODO
}

// - private method
int compute_commands()
{
    //TODO
} 

// - private method
int compute_bias()
{
    //TODO
}

// - private method
int compute_gain()
{
    //TODO
}