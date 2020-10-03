#ifndef DOG_CONTROL_UTILS_INITIALIZER_H
#define DOG_CONTROL_UTILS_INITIALIZER_H

#include "ParamDict.h"

namespace dog_control
{

namespace utils
{

/**
 * @brief BuildParamDict
 * Read options from a LUA file.
 * The options should be listed in a table named "options",
 * for example:
 * --config.lua
 * options = {
 *   opt1 = 0.1,
 *   opt2 = 0.2,
 *   ns1 = {
 *     opt3_in_ns1 = 0.3,
 *     opt4_in_ns1 = 0.4,
 *   },
 *   ...
 * }
 * @param lua_file file name
 * @return params in table "options"
 */
ParamDict BuildParamDict(const std::string& lua_file);

} /* utils */

} /* dog_control */

#endif /* DOG_CONTROL_UTILS_INITIALIZER_H */
