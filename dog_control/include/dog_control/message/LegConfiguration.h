#ifndef DOG_CONTROL_MESSAGE_LEGCONFIGURATION_H
#define DOG_CONTROL_MESSAGE_LEGCONFIGURATION_H

#include "LegName.h"

namespace dog_control
{

namespace message
{

struct LegConfiguration
{
    LegName foot_name;

    double kp;
    double kd;

    // used to decide inverse kinematics
    bool hip_outwards;
    bool knee_outwards;
};

using LegConfigRef = LegConfiguration&;
using LegConfigCRef = const LegConfiguration&;

} /* message */

} /* dog_control */

#endif /* DOG_CONTROL_MESSAGE_LEGCONFIGURATION_H */
