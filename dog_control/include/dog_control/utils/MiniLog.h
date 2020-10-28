#ifndef DOG_CONTROL_UTILS_MINILOG_H
#define DOG_CONTROL_UTILS_MINILOG_H

#include <sstream>

#include "dog_control/utils/ParamDict.h"

namespace dog_control
{

namespace utils
{

enum LOG_LEVEL
{
    INFO = 0,
    DEBUG = 1,
    WARN = 2,
    ERROR = 3,
    FATAL = 4
};

class InfoStream
{
protected:
    using IoOperator = std::ostream& (*)(std::ostream&);
public:
    InfoStream(LOG_LEVEL log_stat);

    ~InfoStream();

    template <typename T>
    inline InfoStream& operator<< (const T& val)
    {
        ss_ << val;

        return *this;
    }

    template <typename T>
    inline InfoStream& operator<< (T* const& pointer)
    {
        if (pointer == nullptr)
            ss_ << "(null)";
        else
            ss_ << pointer;

        return *this;
    }

    inline InfoStream& operator<< (IoOperator pf)
    {
        ss_ << pf;

        return *this;
    }

    InfoStream& operator<< (bool b);

private:
    const LOG_LEVEL log_state_;

    std::ostringstream ss_;
};

} /* utils */

#define LOG(INFO_STATE) utils::InfoStream(utils::INFO_STATE)

#define CHECK(expr) \
    if (expr) \
        ; \
    else \
        LOG(FATAL) \
        << "Check failed in " << __FILE__ \
        << " at line " << __LINE__ \
        << ": in function " << __func__ << ": "

inline double ReadParOrDie(utils::ParamDictCRef dict,
                           const std::string &param)
{
    const utils::ParamDict::const_iterator iter = dict.find(param);

    CHECK(iter != dict.end()) << "Param \"" << param << "\" not found.";

    return iter->second;
}

} /* dog_control */

#endif /* DOG_CONTROL_UTILS_MINILOG_H */
