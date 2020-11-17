#ifndef DOG_CONTROL_PHYSICS_SPATIALTOOLBOX_H
#define DOG_CONTROL_PHYSICS_SPATIALTOOLBOX_H

#include <Eigen/Eigen>

#include "SpatialProperties.h"

namespace dog_control
{

namespace physics
{

namespace spatial
{

/**
 * @brief MotionCrossProduct
 * Motion vector cross product function, copied from MIT cheetah code.
 * Returns a x b. SVec b should be a motion vector.
 */
SVec MotionCrossProduct(const SVec& a, const SVec& b);

/**
 * @brief ForceCrossProduct
 * Force vector cross product function, copied from MIT cheetah code.
 * Returns a x* b. SVec b should be a force vector.
 */
SVec ForceCrossProduct(const SVec& a, const SVec& b);

/**
 * @brief ForceCrossMat
 * Returns a matrix A, which satisfies
 *  A * b = a x* b
 * for all spatial vector b.
 */
SMat ForceCrossMat(const SVec& a);

/**
 * @brief BuildInertia
 * Compute spatial inertia matrix with mass, centre of mass pos
 * and rotation inertia at COM.
 */
SMat BuildInertia(double mass,
                  const Eigen::Vector3d& com,
                  const Eigen::Matrix3d& rot_iner_com);

/**
 * @brief BuildTransform
 * Build spatial transform from frame A to frame B for motion vectors.
 * @param translation   offset of frame B measured in frame A
 * @param rotation      the rotation of B measured in frame A
 * @return              spatial transform from A to B
 */
SMat BuildTransform(const Eigen::Vector3d& translation,
                    const Eigen::Quaterniond& rotation);

/**
 * @brief BuildJointTransform
 * Build spatial transform of a joint
 * @param joint_axis    motion vector satisfies: joint_axis * vq = joint_vel
 * @param joint_type    type of the joint
 * @param q             position of the joint
 * @return              spatial transform from parent link to child link.
 */
SMat BuildJointTransform(const SVec& joint_axis,
                         const JointType joint_type,
                         const double q);

/**
 * @brief MotionTfInverse
 * Inverse a transform matrix for motion vectors.
 * @param motion_tf     transform of motion vectors from frame A to frame B
 * @return              transform from frame B to frame A
 */
SMat MotionTfInverse(const SMat& motion_tf);

/**
 * @brief RotationVel
 * Compute rotational velocity from spatial velocity
 * @param s_vel     spatial velocity
 * @return          rotational velocity
 */
Eigen::Vector3d RotationVel(const SVec& s_vel);

/**
 * @brief LinearVel
 * Compute linear velocity of one point (on the rigid-body),
 * given the rigid-body's spatial velocity
 * @param s_vel     spatial velocity
 * @param pos       the position of the point
 * @return          velocity of the point
 */
Eigen::Vector3d LinearVel(const SVec& s_vel,
                          const Eigen::Vector3d& pos);

/**
 * @brief PointTf
 * Compute a point's coordinate in a new frame.
 * @param Xform     transform matrix
 * @param pos       point's pos in original frame
 * @return          coordinate in new frame
 */
Eigen::Vector3d PointTf(const SMat& Xform,
                        const Eigen::Vector3d& pos);


} /* spatial */

} /* physics */

} /* dog_control */

#endif /* DOG_CONTROL_PHYSICS_SPATIALTOOLBOX_H */
