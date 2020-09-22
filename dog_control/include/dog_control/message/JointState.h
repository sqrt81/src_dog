#ifndef DOG_CONTROL_MESSAGE_JOINTSTATE_H
#define DOG_CONTROL_MESSAGE_JOINTSTATE_H

#include <array>

namespace dog_control
{

namespace message
{

struct SingleJointState
{
    double pos;
    double vel;
    double acc;
    double eff; // effort applied by the joint
};

using JointState3 = std::array<SingleJointState, 3>;
using JointState12 = std::array<SingleJointState, 12>;

struct StampedJointState
{
    double stamp;
    JointState12 joint_state;
};

using JointState3CRef = const JointState3&;
using JointState12CRef = const JointState12&;
using StampedStateCRef = const StampedJointState&;

} /* message */

} /* dog_control */


#endif /* DOG_CONTROL_MESSAGE_JOINTSTATE_H */
