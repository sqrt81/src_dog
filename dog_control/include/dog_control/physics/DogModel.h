#ifndef DOG_CONTROL_PHYSICS_DOGMODEL_H
#define DOG_CONTROL_PHYSICS_DOGMODEL_H

#include "FloatingBaseModel.h"
#include "dog_control/message/JointState.h"
#include "dog_control/message/LegName.h"
#include "dog_control/utils/ParamDict.h"

namespace dog_control
{

namespace physics
{

spatial::FloatingBaseModel BuildDogModel(utils::ParamDictCRef dict);

class DogModel : protected spatial::FloatingBaseModel
{
protected:
    using JointState3 = message::JointState3;
    using LegName = message::LegName;
public:
    void Initialize(utils::ParamDictCRef dict);

    void InverseKinematics(LegName leg_name,
                           const Eigen::Vector3d &foot_pos,
                           JointState3 &joint_pos,
                           bool knee_out, bool hip_out) const;

    using spatial::FloatingBaseModel::SetJointMotionState;
    using spatial::FloatingBaseModel::MassMatrix;
    using spatial::FloatingBaseModel::BiasForces;

    Eigen::Vector3d FootPos(LegName leg_name) const;

    Eigen::Vector3d FootVel(LegName leg_name) const;

    Eigen::Matrix3d JointJacob(LegName leg_name) const;

private:
    double hip_pos_x_; // hip_pos_x_ = hip_pos_x + hip_len_x
    double hip_pos_y_;
    double hip_len_y_;
    double thigh_offset_z_;
    double shin_offset_z_;

    // motor friction
    double friction_;
    double damping_;
    double vel_deadband_;
};

} /* physics */

} /* dog_control */

#endif /* DOG_CONTROL_PHYSICS_DOGMODEL_H */
