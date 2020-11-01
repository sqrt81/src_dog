#ifndef DOG_CONTROL_MESSAGE_ESTIMATORRESULT_H
#define DOG_CONTROL_MESSAGE_ESTIMATORRESULT_H

#include <Eigen/Eigen>

namespace dog_control
{

namespace message
{

struct EstimatorInput
{

};

using EICRef = const EstimatorInput&;

struct EstimatorResult
{
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;

    Eigen::Vector3d linear_vel;
    Eigen::Vector3d rot_vel;

    Eigen::Vector3d linear_acc;

    std::array<Eigen::Vector3d, 4> foot_force;
    std::array<bool, 4> foot_contact;
};

} /* message */

} /* dog_control */

#endif /* DOG_CONTROL_MESSAGE_ESTIMATORRESULT_H */
