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

#ifndef KDL_EIGEN_CONVERSIONS_HPP
#define KDL_EIGEN_CONVERSIONS_HPP
#include <kdl/kdl.hpp>
#include <kdl/kinfam_io.hpp>
#include "kdl/frames.hpp"
#include <kdl/frames_io.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace conversions
{
    // Convert from KDL vector to a 3x1 eigen vector
    Eigen::Vector3d kdl_vector_to_eigen(const KDL::Vector &kdl_vector);

    // Convert from KDL twist to a 6x1 eigen vector/matrix
    Eigen::VectorXd kdl_twist_to_eigen(const KDL::Twist &kdl_twist);

    // Convert from KDL rotation matrix to a 3x3 eigen matrix
    Eigen::Matrix3d rotation_to_eigen(const KDL::Rotation &kdl_matrix);

    // Convert from 3x3 eigen matrix to a KDL rotation matrix
    KDL::Rotation eigen_to_rotation(const Eigen::Matrix3d &eigen_matrix);
}
#endif /* KDL_EIGEN_CONVERSIONS_HPP */