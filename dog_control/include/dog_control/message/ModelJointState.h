#ifndef DOG_CONTROL_MESSAGE_MODELJOINTSTATE_H
#define DOG_CONTROL_MESSAGE_MODELJOINTSTATE_H

#include <vector>
#include <Eigen/Eigen>

namespace dog_control
{

namespace message
{

struct FloatingBaseState
{
    Eigen::Vector3d trans;
    Eigen::Quaterniond rot;

    // note that base velocity is expressed in base frame,
    // not in fixed base (global) frame
    Eigen::Vector3d linear_vel;
    Eigen::Vector3d rot_vel;
};

using FBStateCRef = const FloatingBaseState&;

struct FloatingBaseStateWithAcc
{
    FloatingBaseState state;

    // acceleration is also expressed in base frame.
    Eigen::Vector3d linear_acc;
    Eigen::Vector3d rot_acc;
};

struct StampedFloatingBaseState
{
    double stamp;
    FloatingBaseState state;
};

using StampedFBStateCRef = const StampedFloatingBaseState&;

/**
 * @brief JointState
 * Position and velocity for all the joints.
 * A bit different from "JointState".
 */
struct FloatingBaseJointState
{
    FloatingBaseState base;

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
