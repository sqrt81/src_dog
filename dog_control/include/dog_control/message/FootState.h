#ifndef DOG_CONTROL_MESSAGE_FOOTSTATE_H
#define DOG_CONTROL_MESSAGE_FOOTSTATE_H

#include <Eigen/Eigen>
#include "LegName.h"

namespace dog_control
{

namespace message
{

struct FootState
{
    LegName foot_name;
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
};

using FootStateCRef = const FootState&;

struct LegConfiguration
{
    LegName foot_name;

    double kp;
    double kd;

    Eigen::Vector3d joint_forces;
};

using LegConfigCRef = const LegConfiguration&;

} /* message */

} /* dog_control */


#endif /* DOG_CONTROL_MESSAGE_FOOTSTATE_H */
