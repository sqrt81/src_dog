#include "dog_control/physics/SimDogModel.h"

namespace dog_control
{

namespace physics
{

void SimDogModel::Update(FBJSCRef stat)
{
    FloatingBaseModel::SetJointMotionState(stat);

    ForwardKinematics();

    for (int i = 0; i < 4; i++)
    {
        foot_pos_[i] = EEPos(i);
        foot_vel_[i] = EEVel(i);
    }
}

} /* physics */

} /* dog_control */
