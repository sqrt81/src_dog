#include "dog_control/physics/EigenToolbox.h"

#include "dog_control/utils/Math.h"

namespace dog_control
{

namespace physics
{

Eigen::Matrix3d GeneralInverse(const Eigen::Matrix3d& A)
{
    Eigen::Matrix3d base = A.transpose() * A;
    base.diagonal().array() += utils::precision;
    return base.inverse() * A.transpose();
}

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

Eigen::Vector3d QuatToSO3(const Eigen::Quaterniond &quat)
{
    Eigen::Vector3d so3;
    so3.x() = quat.x();
    so3.y() = quat.y();
    so3.z() = quat.z();

    const double norm = so3.norm();
    const double theta = 2.0 * asin(norm);

    if (abs(theta) > 1e-6)
        so3 *= theta / norm;
    else
        so3.setZero();

    return so3;
}

Eigen::Quaterniond SO3ToQuat(const Eigen::Vector3d &so3)
{
    Eigen::Quaterniond quat;
    const double norm = so3.norm() / 2;
    Eigen::Vector3d unit;

    if (abs(norm) > 1e-6)
    {
        unit = so3 / norm * 2;

        quat.w() = cos(norm);
        quat.vec() = unit * sin(norm);
    }
    else
        quat.setIdentity();

    return quat;
}

} /* physics */

} /* dog_control */
