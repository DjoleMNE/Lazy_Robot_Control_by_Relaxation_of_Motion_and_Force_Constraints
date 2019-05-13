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

#ifndef GEOMETRY_UTILS_HPP
#define GEOMETRY_UTILS_HPP
#include <state_specification.hpp>
#include <Eigen/Geometry>
#include <kdl_eigen_conversions.hpp>
#include <unistd.h>
#include <cmath>
#include <stdlib.h>     /* abs */

namespace geometry
{   
    /**
     * Calculate logarithmic map given rotation matrix. 
     * Sources - Combined from: 
     * http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToAngle/index.htm
     * https://github.com/NxRLab/ModernRobotics/blob/master/packages/MATLAB/mr/MatrixLog3.m
    */					   
    KDL::Vector log_map_so3(const KDL::Rotation &matrix);

    /**
     * Calculate logarithmic map given translation vector and
     * angular twist as result of Log_SO(e). 
     * Sources - Combined from: 
     * S. Grazioso et al., "From Differential Geometry of Curves to Helical 
     * Kinematics of Continuum Robots Using Exponential Mapping"
     * https://github.com/NxRLab/ModernRobotics/blob/master/packages/MATLAB/mr/MatrixLog6.m
     * Function takes non-normalized vectors of angular twist and translation.
    */					   
    KDL::Vector log_map_r3(const KDL::Vector &translation,
                           const KDL::Vector &angular_twist);

    /**
     * Calculate logarithmic map given a transformation matrix.
     * Sources - Combined from: 
     * S. Grazioso et al., "From Differential Geometry of Curves to Helical 
     * Kinematics of Continuum Robots Using Exponential Mapping"
     * https://github.com/NxRLab/ModernRobotics/blob/master/packages/MATLAB/mr/MatrixLog6.m
    */					   
    KDL::Twist log_map_se3(const KDL::Frame &pose);

    /**
     * Calculate exponential map for angular part of the given screw twist. 
     * Given twist vector should NOT be normalized!
     * If rotation is too small, rotation matrix will be set to identity
    */
    KDL::Rotation exp_map_so3(const KDL::Vector &current_twist);
    /**
     * Calculate exponential map for linear part of the given screw twist.
     * Given twist vector should NOT be normalized!
    */
    KDL::Vector exp_map_r3(const KDL::Twist &current_twist);
    /**
     * Calculate exponential map for both linear and angular parts 
     * of the given screw twist. Given Twist should NOT be normalized!
     * If rotation is too small, rotation matrix will be set to identity
    */
    KDL::Frame exp_map_se3(const KDL::Twist &current_twist);


    //Converts a 3D vector to an skew matrix representation
    KDL::Rotation skew_matrix(const KDL::Vector &vector);

    //Scale a 3x3 matrix with a scalar number
    KDL::Rotation scale_matrix(const KDL::Rotation &matrix,
                               const double scale);

    // Performs element-wise additions of two matrices
    KDL::Rotation matrix_addition(const KDL::Rotation &matrix_1,
                                  const KDL::Rotation &matrix_2);
    /** 
     * Perform parameterization of rot twist if the angle is > PI 
     * to avoid singularties in exponential maps.
     * Code based on: 
     * F. Sebastian Grassia, "Practical Parameterization of Rotations 
     * Using the Exponential Map" paper.
    */
    bool rescale_angular_twist(KDL::Vector &rot_twist);

    /** 
     * Solving Generalized/constrained Procrustes problem i.e. 
     * bringing computed matrix back to the SO(3) manifold.
     * source: https://ieeexplore.ieee.org/document/88573
    */
    void orthonormalize_rot_matrix(KDL::Rotation &rot_matrix);

    // Determine if a matrix is rotational or not
    bool is_rotation_matrix(const KDL::Rotation &m);

    // Compute determinant of a 3x3 matrix
    double determinant(const KDL::Rotation &m);

    // Compute distance of the matrix from the SO(3) manifold
    double distance_to_so3(const Eigen::Matrix3d &matrix);

    /**
     * Multiply jacobian transpose with a wrench to get resulting joint torques
     * Inverse Force Kinematics
     */
    Eigen::VectorXd ik_force(const KDL::Jacobian &jac, const KDL::Wrench &wrench);
} 
#endif /* GEOMETRY_UTILS_HPP */