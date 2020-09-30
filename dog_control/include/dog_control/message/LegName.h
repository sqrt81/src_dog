#ifndef DOG_CONTROL_MESSAGE_LEGNAME_H
#define DOG_CONTROL_MESSAGE_LEGNAME_H

namespace dog_control
{

namespace message
{

enum LegName
{
    FL = 0,
    FR = 1,
    BL = 2,
    BR = 3,
};

#define VALID_LEGNAME(name) (static_cast<unsigned>(name) < 4)

} /* message */

} /* dog_control */


#endif /* DOG_CONTROL_MESSAGE_LEGNAME_H */
