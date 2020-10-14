#ifndef DOG_CONTROL_HARDWARE_HARDWAREBASE
#define DOG_CONTROL_HARDWARE_HARDWAREBASE

#include "dog_control/message/Imu.h"
#include "dog_control/message/JointState.h"
#include "dog_control/message/MotorCommand.h"

#include "dog_control/utils/ParamDict.h"

namespace dog_control
{

namespace hardware
{

/**
 * @brief The HardwareBase class
 * This is the virtual basic class for hardware interface.
 * All derived classes should implement functions which
 * communicate with the hardware.
 */
class HardwareBase
{
protected:
    using MotorCommandCRef = message::MotorCommandCRef;
    using StampedJointState = message::StampedJointState;
    using StampedImu = message::StampedImu;

public:
    HardwareBase() = default;

    ~HardwareBase() = default;

    virtual void Initialize(utils::ParamDictCRef dict) = 0;

    /**
     * @brief Spin
     * A blocking, simple-event loop.
     */
    virtual void Spin() = 0;

    virtual void PublishCommand(MotorCommandCRef command) const = 0;

    inline StampedJointState GetJointState() const
    {
        return joint_state_;
    }

    inline StampedImu GetImuInfo() const
    {
        return imu_info_;
    }

protected:
    StampedImu imu_info_;
    StampedJointState joint_state_;
};

} /* hardware */

} /* dog_control */

#endif /* DOG_CONTROL_HARDWARE_HARDWAREBASE */
