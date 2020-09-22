#ifndef DOG_CONTROL_MESSAGE_MOTORCOMMAND_H
#define DOG_CONTROL_MESSAGE_MOTORCOMMAND_H

#include <array>

namespace dog_control
{

namespace message
{

struct SingleMotorCommand
{
    double x_desired;
    double kp;
    double v_desired;
    double kd;
    double torq;
};

using MotorCommand = std::array<SingleMotorCommand, 12>;

using MotorCommandCRef = const MotorCommand&;

} /* message */

} /* dog_control */


#endif /* DOG_CONTROL_MESSAGE_MOTORCOMMAND_H */
