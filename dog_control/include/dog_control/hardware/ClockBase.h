#ifndef DOG_CONTROL_HARDWARE_CLOCKBASE_H
#define DOG_CONTROL_HARDWARE_CLOCKBASE_H


namespace dog_control
{

namespace hardware
{

/**
 * @brief The ClockBase class
 * ClockBase is base class for clocks.
 * Clocks provides timestamps for other classes,
 * and keep them in sync.
 */
class ClockBase
{
public:
    ClockBase() = default;

    double Time()
    {
        return stamp_;
    }

    virtual void Update() = 0;

protected:
    double stamp_;
};

} /* hardware */

} /* dog_control */

#endif /* DOG_CONTROL_HARDWARE_CLOCKBASE_H */
