#include "dog_control/utils/Initializer.h"
#include <lua.hpp>

namespace dog_control
{

namespace utils
{

namespace
{

class LuaReader
{
public:
    LuaReader(const std::string& lua_file)
    {
        l_state = luaL_newstate();
        luaL_openlibs(l_state);

        // append current lua file path to lua path
        const std::string file_path
                = lua_file.substr(0, lua_file.rfind('/'));

        /** add file location to lua search path
         *  see https://stackoverflow.com/questions/4125971
         */
        lua_getglobal(l_state, "package");
        lua_getfield(l_state, -1, "path");
        std::string cur_path = lua_tostring(l_state, -1);
        cur_path.append(";" + file_path + "/?.lua");
        lua_pop(l_state, 1);
        lua_pushstring(l_state, cur_path.c_str());
        lua_setfield(l_state, -2, "path");
        lua_pop(l_state, 1);

        file_ready = !luaL_dofile(l_state, lua_file.c_str());
    }

    ~LuaReader()
    {
        if (file_ready)
            lua_close(l_state);
    }

    ParamDict ReadALL() const;

private:
    void ReadWithNamespace(ParamDict& params, const std::string& ns) const;

    bool file_ready;
    lua_State* l_state;
};

ParamDict LuaReader::ReadALL() const
{
    ParamDict params;

    if (!file_ready)
        return params;

    // Assume all options are inside a table named "options"
    lua_getglobal(l_state, "options");

    if (lua_istable(l_state, -1))
    {
        ReadWithNamespace(params, "");
    }

    return params;
}

void LuaReader::ReadWithNamespace(ParamDict &params, const std::string &ns) const
{
    int t_idx = lua_gettop(l_state);
    lua_pushnil(l_state);

    while (lua_next(l_state, t_idx))
    {
        // index -2 --> key
        // index -1 --> value
        if (lua_isstring(l_state, -2))
        {
            const std::string key
                    = ns + lua_tostring(l_state, -2);

            if (lua_isnumber(l_state, -1))
            {
                double value = lua_tonumber(l_state, -1);
                params.insert(std::make_pair(key, value));
            }
            else if (lua_istable(l_state, -1))
            {
                ReadWithNamespace(params, key + "/");
            }
        }

        lua_pop(l_state, 1);
    }
}

} /* anonymous namespace */

ParamDict BuildParamDict(const std::string& lua_file)
{
    LuaReader reader(lua_file);

    return reader.ReadALL();
}

} /* utils */

} /* dog_control */
