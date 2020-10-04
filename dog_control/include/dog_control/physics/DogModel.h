#ifndef DOG_CONTROL_PHYSICS_DOGMODEL_H
#define DOG_CONTROL_PHYSICS_DOGMODEL_H

#include "FloatingBaseModel.h"
#include "dog_control/estimator/EstimatorBase.h"
#include "dog_control/message/JointState.h"
#include "dog_control/message/LegName.h"
#include "dog_control/hardware/HardwareBase.h"
#include "dog_control/utils/ParamDict.h"

#include <boost/weak_ptr.hpp>

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
    using FullJacobMat = Eigen::Matrix<double, 3, 18>;
    using JointForce = Eigen::Matrix<double, 12, 1>;

    void Initialize(utils::ParamDictCRef dict);

    void ConnectHardware(boost::shared_ptr<hardware::HardwareBase> hw);
    void ConnectEstimator(boost::shared_ptr<estimator::EstimatorBase> est);

    void InverseKinematics(LegName leg_name,
                           const Eigen::Vector3d &foot_pos,
                           JointState3 &joint_pos,
                           bool knee_out, bool hip_out) const;

//    using spatial::FloatingBaseModel::SetJointMotionState;
    using spatial::FloatingBaseModel::ForwardKinematics;
    using spatial::FloatingBaseModel::MassMatrix;
    using spatial::FloatingBaseModel::BiasForces;

    Eigen::Vector3d FootPos(LegName leg_name);

    Eigen::Vector3d FootVel(LegName leg_name);

    Eigen::Matrix3d JointJacob(LegName leg_name);

    FullJacobMat FullJacob(LegName leg_name);

    JointForce Friction() const;

    void Update();

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

    boost::weak_ptr<hardware::HardwareBase> hw_ptr_;
    boost::weak_ptr<estimator::EstimatorBase> est_ptr_;
};

} /* physics */

} /* dog_control */

#endif /* DOG_CONTROL_PHYSICS_DOGMODEL_H */
