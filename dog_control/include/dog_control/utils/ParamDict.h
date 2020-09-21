#ifndef DOG_CONTROL_UTILS_PARAMDICT_H
#define DOG_CONTROL_UTILS_PARAMDICT_H

#include <map>
#include <string>

namespace dog_control
{

namespace utils
{

typedef std::map<std::string, double> ParamDict;
typedef const ParamDict& ParamDictCRef;

} /* utils */

} /* dog_control */

#endif // PARAMDICT_H
