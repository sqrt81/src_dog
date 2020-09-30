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

struct FootConfiguration
{
    LegName foot_name;

    double kp;
    double kd;

    /**
     * foot_force is the reaction force that ground applies to this foot.
     */
    Eigen::Vector3d foot_force;
};

using FootConfigCRef = const FootConfiguration&;

} /* message */

} /* dog_control */


#endif /* DOG_CONTROL_MESSAGE_FOOTSTATE_H */
