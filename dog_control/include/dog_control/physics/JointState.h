#ifndef DOG_CONTROL_PHYSICS_JOINTSTATE_H
#define DOG_CONTROL_PHYSICS_JOINTSTATE_H

#include <array>

namespace dog_control
{

namespace physics
{

struct SingleJointState
{
    double pos;
    double vel;
    double acc;
    double eff; // effort applied by the joint
};

typedef std::array<SingleJointState, 3> JointState3;
typedef std::array<SingleJointState, 12> JointState12;

typedef const JointState3& JointState3CRef;
typedef const JointState12& JointState12CRef;

} /* physics */

} /* dog_control */


#endif // DOG_CONTROL_PHYSICS_JOINTSTATE_H
