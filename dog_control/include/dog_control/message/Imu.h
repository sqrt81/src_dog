#ifndef DOG_CONTROL_MESSAGE_IMU_H
#define DOG_CONTROL_MESSAGE_IMU_H

#include <Eigen/Eigen>

namespace dog_control
{

namespace message
{

struct Imu
{
    Eigen::Vector3d acceleration;
    Eigen::Vector3d angular_speed;
    Eigen::Quaterniond rotation;
};

struct StampedImu
{
    double stamp;
    Imu imu_data;
};

using ImuCRef = const Imu&;
using StampedImuCRef = const StampedImu&;

} /* message */

} /* dog_control */


#endif /* DOG_CONTROL_MESSAGE_IMU_H */
