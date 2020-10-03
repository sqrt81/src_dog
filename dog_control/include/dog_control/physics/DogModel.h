#ifndef DOG_CONTROL_PHYSICS_DOGMODEL_H
#define DOG_CONTROL_PHYSICS_DOGMODEL_H

#include "FloatingBaseModel.h"
#include "dog_control/utils/ParamDict.h"

namespace dog_control
{

namespace physics
{

namespace spatial
{

FloatingBaseModel BuildDogModel(utils::ParamDictCRef dict);

} /* spatial */

} /* physics */

} /* dog_control */

#endif /* DOG_CONTROL_PHYSICS_DOGMODEL_H */
