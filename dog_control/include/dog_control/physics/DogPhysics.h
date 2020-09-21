#ifndef DOG_CONTROL_PHYSICS_DOGPHYSICS_H
#define DOG_CONTROL_PHYSICS_DOGPHYSICS_H

#include "dog_control/utils/ParamDict.h"
#include "JointState.h"

#include <Eigen/Eigen>

namespace dog_control
{

namespace physics
{

class DogPhysics
{
public:
    DogPhysics() = default;

    void Initialize(utils::ParamDictCRef dict);

    Eigen::Vector3d ForwardKinematics(JointState3CRef joint_pos);

    void InverseKinematics(const Eigen::Vector3d& foot_pos,
                           JointState3& joint_pos,
                           bool knee_out, bool hip_out);

    void BuildDynamicsMatrixs(JointState3CRef joint_stat,
                              Eigen::Matrix3d& H,
                              Eigen::Matrix3d& C,
                              Eigen::Matrix3d& G);

    Eigen::Vector3d ComputeTorque(JointState3CRef joint_stat);

private:
    Eigen::Vector3d ComputeFriction(JointState3CRef joint_stat);

    // length
    double hip_pos_x_;
    double hip_pos_y_;
    double hip_len_y_;
    double thigh_offset_z_;
    double shin_offset_z_;

    // com pos
    double com_hip_y_;
    double com_thigh_y_;
    double com_thigh_z_;
    double com_shin_z_;

    // mass
    double hip_mass_;
    double thigh_mass_;
    double shin_mass_;

    // inertia
    double I1xx_; // I1XX is inertia for hip
    double I2xx_; // I2XX is for thigh
    double I2yy_;
    double I2yz_;
    double I2zz_;
    double I3xx_; // I3XX is for shin
    double I3yy_;
    double I3zz_;
};

} /* physics */

} /* dog_control */

#endif /* DOG_CONTROL_PHYSICS_DOGPHYSICS_H */
