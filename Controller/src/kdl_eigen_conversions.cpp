#include <kdl_eigen_conversions.hpp>

namespace conversions
{
    // Convert from KDL vector to a 3x1 eigen vector
    Eigen::Vector3d kdl_vector_to_eigen(const KDL::Vector &kdl_vector)
    {
        return Eigen::Vector3d(kdl_vector(0), kdl_vector(1), kdl_vector(2));
    }

    // Convert from KDL twist to a 6x1 eigen vector
    Eigen::VectorXd kdl_twist_to_eigen(const KDL::Twist &kdl_twist)
    {
        return (Eigen::VectorXd(6) << kdl_twist(0), kdl_twist(1), kdl_twist(2),
                                      kdl_twist(3), kdl_twist(4), kdl_twist(5)).finished();
    }

    Eigen::Matrix3d rotation_to_eigen(const KDL::Rotation &kdl_matrix)
    {
        return (Eigen::Matrix3d() << kdl_matrix.data[0], kdl_matrix.data[1], kdl_matrix.data[2],
                                     kdl_matrix.data[3], kdl_matrix.data[4], kdl_matrix.data[5],
                                     kdl_matrix.data[6], kdl_matrix.data[7], kdl_matrix.data[8]).finished();
    }

    KDL::Rotation eigen_to_rotation(const Eigen::Matrix3d &eigen_matrix)
    {
        return KDL::Rotation(eigen_matrix(0, 0), eigen_matrix(0, 1), eigen_matrix(0, 2),
                             eigen_matrix(1, 0), eigen_matrix(1, 1), eigen_matrix(1, 2),
                             eigen_matrix(2, 0), eigen_matrix(2, 1), eigen_matrix(2, 2));
    }
}