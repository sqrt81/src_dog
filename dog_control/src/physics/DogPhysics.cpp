#include "dog_control/physics/DogPhysics.h"

#include <cassert>

#define READ_PARAM_OR_DIE(dict, param, ns) \
    do \
    { \
        const utils::ParamDict::const_iterator iter = dict.find(#ns "/" #param);\
        assert(iter != dict.end()); \
        param ## _ = iter->second; \
    } \
    while(false)


namespace dog_control
{

namespace physics
{

void DogPhysics::Initialize(utils::ParamDictCRef dict)
{
    double hip_len_x_;

    READ_PARAM_OR_DIE(dict, hip_pos_x, physics);
    READ_PARAM_OR_DIE(dict, hip_pos_y, physics);
    READ_PARAM_OR_DIE(dict, hip_len_x, physics);
    READ_PARAM_OR_DIE(dict, hip_len_y, physics);
    hip_pos_x_ += hip_len_x_;
    READ_PARAM_OR_DIE(dict, thigh_offset_z, physics);
    READ_PARAM_OR_DIE(dict, shin_offset_z, physics);

    READ_PARAM_OR_DIE(dict, com_hip_y, physics);
    READ_PARAM_OR_DIE(dict, com_thigh_y, physics);
    READ_PARAM_OR_DIE(dict, com_thigh_z, physics);
    READ_PARAM_OR_DIE(dict, com_shin_z, physics);

}

} /* physics */

} /* dog_control */
