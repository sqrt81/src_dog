#ifndef DOG_CONTROL_UTILS_PARAMDICT_H
#define DOG_CONTROL_UTILS_PARAMDICT_H

#include <map>
#include <string>

namespace dog_control
{

namespace utils
{

#define PARAM_WITH_NS(param, ns) #ns "/" #param

using ParamDict = std::map<std::string, double>;
using ParamDictCRef = const ParamDict&;

} /* utils */

} /* dog_control */

#endif // PARAMDICT_H
