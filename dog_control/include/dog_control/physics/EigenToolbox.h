#ifndef DOG_CONTROL_PHYSICS_EIGENTOOLBOX_H
#define DOG_CONTROL_PHYSICS_EIGENTOOLBOX_H

#include <Eigen/Eigen>

namespace dog_control
{

namespace physics
{

/**
 * @brief ToLowerMatrix
 * Get a matrix M that for any vector a,
 * upper_vec X a = M * a.
 */
Eigen::Matrix3d ToLowerMatrix(const Eigen::Vector3d &upper_vec);

/**
 * @brief ToUpperVector
 * Inverse function of "ToLowerMatrix"
 */
Eigen::Vector3d ToUpperVector(const Eigen::Matrix3d &lower_mat);

/**
 * @brief RightQuatToMatrix
 * Convert a quaternion to a matrix.
 * Suppose q1 and q2 are two quaternions.
 * Then the combination of q1 and q2 can be expressed by
 *  q1 * q2 = Q * [w1 x1 y1 z1].transpose(),
 * where Q is a 4x4 matrix depends only on q2,
 * and [w1 x1 y1 z1] are coefficients of q1.
 * This function computes matrix Q from quaternion q2.
 */
Eigen::Matrix4d RightQuatToMatrix(const Eigen::Quaterniond &q);

/**
 * @brief QuatToSO3
 * Convert a quaternion to angle-axis form.
 * The returned vector V's norm equals to the angle,
 * and it is parallel to the axis (if its norm is not zero).
 */
Eigen::Vector3d QuatToSO3(const Eigen::Quaterniond &quat);

/**
 * @brief SO3ToQuat
 * Inverse function of QuatToSO3.
 */
Eigen::Quaterniond SO3ToQuat(const Eigen::Vector3d &so3);

} /* physics */

} /* dog_control */

#endif /* DOG_CONTROL_PHYSICS_EIGENTOOLBOX_H */
