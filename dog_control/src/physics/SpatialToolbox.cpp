#include "dog_control/physics/SpatialToolbox.h"

#include <cassert>

namespace dog_control
{

namespace physics
{

namespace spatial
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

SMat BuildInertia(double mass,
                  const Eigen::Vector3d& com,
                  const Eigen::Matrix3d &rot_iner_com)
{
    SMat inertia;
    const Eigen::Matrix3d c_lower = ToLowerMatrix(com);

    inertia.topLeftCorner(3, 3)
            = rot_iner_com + c_lower * c_lower.transpose() * mass;
    inertia.topRightCorner(3, 3) = c_lower * mass;
    inertia.bottomLeftCorner(3, 3) = c_lower.transpose() * mass;
    inertia.bottomRightCorner(3, 3) = Eigen::Matrix3d::Identity() * mass;

    return inertia;
}

SMat BuildTransform(const Eigen::Vector3d& translation,
                    const Eigen::Quaterniond& rotation)
{
    // Note: here the rotation is not a part of vector translation matrix,
    // but a relation between two frames.
    // So, a vector V in A, should be transformed to
    // rotation.inverse() * (V - translation)
    // instead of
    // rotation * (V - translation).
    const Eigen::Matrix3d rot = rotation.conjugate().toRotationMatrix();
    const Eigen::Matrix3d trans = ToLowerMatrix(translation);
    SMat transform;
    transform.topLeftCorner(3, 3) = rot;
    transform.topRightCorner(3, 3) = Eigen::Matrix3d::Zero();
    transform.bottomLeftCorner(3, 3) = - rot * trans;
    transform.bottomRightCorner(3, 3) = rot;

    return transform;
}

SMat BuildJointTransform(const SVec& joint_axis,
                         const JointType joint_type,
                         const double q)
{
    Eigen::Quaterniond rot;
    Eigen::Vector3d trans;

    switch(joint_type)
    {
    case revolute:
    {
        double norm = joint_axis.topRows(3).norm();
        rot = Eigen::AngleAxisd(q * norm, joint_axis.topRows(3) / norm);
        trans = Eigen::Vector3d::Zero();
    }
        break;
    case floating:
        assert(false /* floating base should use BuildTransform */);
    default:
        assert(false /* unknown joint type */);
    }

    return BuildTransform(trans, rot);
}

SMat MotionTfInverse(const SMat& motion_tf)
{
    SMat inverse;
    const Eigen::Matrix3d rot_t
            = motion_tf.topLeftCorner(3, 3).transpose();
    inverse.topLeftCorner(3, 3) = rot_t;
    inverse.topRightCorner(3, 3) = Eigen::Matrix3d::Zero();
    inverse.bottomLeftCorner(3, 3)
            = - rot_t * motion_tf.bottomLeftCorner(3, 3) * rot_t;
    inverse.bottomRightCorner(3, 3) = rot_t;

    return inverse;
}

Eigen::Vector3d RotationVel(const SVec& s_vel)
{
    return s_vel.topRows(3);
}

Eigen::Vector3d LinearVel(const SVec& s_vel,
                          const Eigen::Vector3d &pos)
{
    const Eigen::Vector3d rot_vel = s_vel.topRows(3);
    return s_vel.bottomRows(3) + rot_vel.cross(pos);
}

Eigen::Vector3d PointTf(const SMat& Xform,
                        const Eigen::Vector3d& pos)
{
    const Eigen::Matrix3d rotation = Xform.topLeftCorner(3, 3);
    const Eigen::Vector3d translation
            = - ToUpperVector(rotation.transpose()
                              * Xform.bottomLeftCorner(3, 3));

    return rotation * (pos - translation);
}

} /* spatial */

} /* physics */

} /* dog_control */
