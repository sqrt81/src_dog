#include "dog_control/utils/MiniLog.h"

#include <iostream>

namespace dog_control
{

namespace utils
{

InfoStream::InfoStream(LOG_LEVEL log_stat)
 : log_state_(log_stat)
{

}
InfoStream::~InfoStream()
{
    switch (log_state_)
    {
    case INFO:
        std::cout << "[INFO]    " << ss_.str() << std::endl;
        break;
    case DEBUG:
        std::cout << "\033[47;30m[DEBUG]   " << ss_.str()
                  << "\033[0m" << std::endl;
        break;
    case WARN:
        std::cout << "\033[33m[WARN]    " << ss_.str()
                  << "\033[0m" << std::endl;
        break;
    case ERROR:
        std::cerr << "\033[31m[ERROR]   " << ss_.str()
                  << "\033[0m" << std::endl;
        break;
    case FATAL:
        std::cerr << "\033[31m[FATAL]   " << ss_.str()
                  << "\033[0m" << std::endl;
        abort();
    default:
        break;
    }
}

InfoStream& InfoStream::operator<< (bool b)
{
    ss_ << (b ? "true" : "false");

    return *this;
}

} /* utils */

} /* dog_control */
