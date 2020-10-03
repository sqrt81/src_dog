#ifndef DOG_CONTROL_MESSAGE_MODELJOINTSTATE_H
#define DOG_CONTROL_MESSAGE_MODELJOINTSTATE_H

#include <vector>
#include <Eigen/Eigen>

namespace dog_control
{

namespace message
{

/**
 * @brief JointState
 * Position and velocity for all the joints.
 * A bit different from "JointState".
 */
struct FloatingBaseJointState
{
    Eigen::Vector3d base_trans;
    Eigen::Quaterniond base_rot;

    // note that base velocity is measured in base frame,
    // not in fixed base (global) frame
    Eigen::Vector3d base_linear_vel;
    Eigen::Vector3d base_rot_vel;

    std::vector<double> q;
    std::vector<double> dq;
};

using FBJSCRef = const FloatingBaseJointState&;

struct FloatingBaseJointStateDerivative
{
    Eigen::Matrix<double, 6, 1> base_acc;

    std::vector<double> ddq;
};

using FBJSDCRef = const FloatingBaseJointStateDerivative&;

} /* message */

} /* dog_control */


#endif /* DOG_CONTROL_MESSAGE_MODELJOINTSTATE_H */
