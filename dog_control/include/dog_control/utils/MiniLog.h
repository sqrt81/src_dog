#ifndef DOG_CONTROL_UTILS_MINILOG_H
#define DOG_CONTROL_UTILS_MINILOG_H

#include <sstream>

namespace dog_control
{

namespace utils
{

enum LOG_LEVEL
{
    INFO = 0,
    WARN = 1,
    ERROR = 2,
    FATAL = 3
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
        LOG(FATAL)

#define CHECK_FATAL(expr) \
    if (expr) \
        ; \
    else \
        LOG(FATAL)

} /* dog_control */

#endif /* DOG_CONTROL_UTILS_MINILOG_H */
