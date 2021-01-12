#include "dog_control/physics/SpatialToolbox.h"
#include "dog_control/physics/EigenToolbox.h"
#include "dog_control/utils/MiniLog.h"

namespace dog_control
{

namespace physics
{

namespace spatial
{

SVec MotionCrossProduct(const SVec& a, const SVec& b)
{
    SVec mv;

    mv << a(1) * b(2) - a(2) * b(1),
          a(2) * b(0) - a(0) * b(2),
          a(0) * b(1) - a(1) * b(0),
          a(1) * b(5) - a(2) * b(4) + a(4) * b(2) - a(5) * b(1),
          a(2) * b(3) - a(0) * b(5) - a(3) * b(2) + a(5) * b(0),
          a(0) * b(4) - a(1) * b(3) + a(3) * b(1) - a(4) * b(0);

    return mv;
}

SVec ForceCrossProduct(const SVec& a, const SVec& b)
{
    SVec mv;

    mv << b(2) * a(1) - b(1) * a(2) - b(4) * a(5) + b(5) * a(4),
          b(0) * a(2) - b(2) * a(0) + b(3) * a(5) - b(5) * a(3),
          b(1) * a(0) - b(0) * a(1) - b(3) * a(4) + b(4) * a(3),
          b(5) * a(1) - b(4) * a(2),
          b(3) * a(2) - b(5) * a(0),
          b(4) * a(0) - b(3) * a(1);

    return mv;
}

SMat ForceCrossMat(const SVec& a)
{
    SMat mat;

    mat.topLeftCorner<3, 3>() = ToLowerMatrix(a.head<3>());
    mat.topRightCorner<3, 3>() = ToLowerMatrix(a.tail<3>());
    mat.bottomLeftCorner<3, 3>().setZero();
    mat.bottomRightCorner<3, 3>() = ToLowerMatrix(a.head<3>());

    return mat;
}

SMat BuildInertia(double mass,
                  const Eigen::Vector3d& com,
                  const Eigen::Matrix3d &rot_iner_com)
{
    SMat inertia;
    const Eigen::Matrix3d c_lower = ToLowerMatrix(com);

    inertia.topLeftCorner<3, 3>()
            = rot_iner_com + c_lower * c_lower.transpose() * mass;
    inertia.topRightCorner<3, 3>() = c_lower * mass;
    inertia.bottomLeftCorner<3, 3>() = c_lower.transpose() * mass;
    inertia.bottomRightCorner<3, 3>() = Eigen::Matrix3d::Identity() * mass;

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
    transform.topLeftCorner<3, 3>() = rot;
    transform.topRightCorner<3, 3>() = Eigen::Matrix3d::Zero();
    transform.bottomLeftCorner<3, 3>() = - rot * trans;
    transform.bottomRightCorner<3, 3>() = rot;

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
        const double norm = joint_axis.topRows<3>().norm();
        rot = Eigen::AngleAxisd(q * norm, joint_axis.topRows<3>() / norm);
        trans = Eigen::Vector3d::Zero();
    }
        break;
    case floating:
        LOG(FATAL) << "[Build Joint Transform] "
                      "Floating base should use BuildTransform.";
    default:
        LOG(FATAL) << "[Build Joint Transform] "
                      "Unknown joint type.";
    }

    return BuildTransform(trans, rot);
}

SMat BuildJointTransformDiff(const SVec& joint_axis,
                             const JointType joint_type,
                             const double q)
{
    Eigen::Quaterniond rot;
    Eigen::Vector3d trans;

    switch(joint_type)
    {
    case revolute:
    {
        const double norm = joint_axis.topRows<3>().norm();
        rot = Eigen::AngleAxisd(q * norm, joint_axis.topRows<3>() / norm);
        trans = Eigen::Vector3d::Zero();
    }
        break;
    case floating:
        LOG(FATAL) << "[Build Joint Transform Diff] "
                      "Floating base can't be differentiated.";
    default:
        LOG(FATAL) << "[Build Joint Transform Diff] "
                      "Unknown joint type.";
    }

    const Eigen::Matrix3d rot_mat
            = ToLowerMatrix(joint_axis.topRows<3>()) * rot.toRotationMatrix();
    SMat transform = SMat::Zero();
    transform.topLeftCorner<3, 3>() = rot_mat.transpose();
    transform.bottomRightCorner<3, 3>() = rot_mat.transpose();

    return transform;
}

SMat MotionTfInverse(const SMat& motion_tf)
{
    SMat inverse;
    const Eigen::Matrix3d rot_t
            = motion_tf.topLeftCorner<3, 3>().transpose();
    inverse.topLeftCorner<3, 3>() = rot_t;
    inverse.topRightCorner<3, 3>() = Eigen::Matrix3d::Zero();
    inverse.bottomLeftCorner<3, 3>()
            = - rot_t * motion_tf.bottomLeftCorner<3, 3>() * rot_t;
    inverse.bottomRightCorner<3, 3>() = rot_t;

    return inverse;
}

Eigen::Vector3d RotationVel(const SVec& s_vel)
{
    return s_vel.topRows<3>();
}

Eigen::Vector3d LinearVel(const SVec& s_vel,
                          const Eigen::Vector3d &pos)
{
    const Eigen::Vector3d rot_vel = s_vel.topRows<3>();
    return s_vel.bottomRows<3>() + rot_vel.cross(pos);
}

Eigen::Vector3d PointTf(const SMat& Xform,
                        const Eigen::Vector3d& pos)
{
    const Eigen::Matrix3d rotation = Xform.topLeftCorner<3, 3>();
    const Eigen::Vector3d translation
            = - ToUpperVector(rotation.transpose()
                              * Xform.bottomLeftCorner<3, 3>());

    return rotation * (pos - translation);
}

} /* spatial */

} /* physics */

} /* dog_control */
