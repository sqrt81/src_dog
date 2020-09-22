#ifndef DOG_CONTROL_HARDWARE_SIMULATEDHARDWARE
#define DOG_CONTROL_HARDWARE_SIMULATEDHARDWARE

#include "dog_control/hardware/HardwareBase.h"

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>

namespace dog_control
{

namespace hardware
{

class SimulatedHardware : public HardwareBase
{
protected:
    using HardwareBase::MotorCommandCRef;

public:
    SimulatedHardware();

    ~SimulatedHardware();

    void Initialize(utils::ParamDictCRef dict);

    void Spin();

    void PublishCommand(MotorCommandCRef command) const;

private:
    void ImuCb(sensor_msgs::ImuConstPtr imu);
    void JointStateCb(sensor_msgs::JointStateConstPtr js);

    std::map<std::string, int> motor_index_;
    ros::NodeHandle nh_;
    ros::Publisher joint_cmd_pub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber joint_state_sub_;
};

} /* hardware */

} /* dog_control */

#endif /* DOG_CONTROL_HARDWARE_SIMULATEDHARDWARE */
