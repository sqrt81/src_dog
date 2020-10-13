#include "dog_control/physics/EigenToolbox.h"

namespace dog_control
{

namespace physics
{

Eigen::Matrix3d ToLowerMatrix(const Eigen::Vector3d& upper_vec)
{
    Eigen::Matrix3d mat;

    mat(0, 0) =               0;
    mat(0, 1) = - upper_vec.z();
    mat(0, 2) =   upper_vec.y();
    mat(1, 0) =   upper_vec.z();
    mat(1, 1) =               0;
    mat(1, 2) = - upper_vec.x();
    mat(2, 0) = - upper_vec.y();
    mat(2, 1) =   upper_vec.x();
    mat(2, 2) =               0;

    return mat;
}

Eigen::Vector3d ToUpperVector(const Eigen::Matrix3d& lower_mat)
{
    return Eigen::Vector3d(lower_mat(2, 1) - lower_mat(1, 2),
                           lower_mat(0, 2) - lower_mat(2, 0),
                           lower_mat(1, 0) - lower_mat(0, 1)) / 2;
}

Eigen::Matrix4d RightQuatToMatrix(const Eigen::Quaterniond &q)
{
    Eigen::Matrix4d quat_mat;
    quat_mat << q.w(), - q.x(), - q.y(), - q.z(),
                q.x(),   q.w(), - q.z(),   q.y(),
                q.y(),   q.z(),   q.w(), - q.x(),
                q.z(), - q.y(),   q.x(),   q.w();

    return quat_mat;
}

} /* physics */

} /* dog_control */
