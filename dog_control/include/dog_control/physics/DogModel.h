#ifndef DOG_CONTROL_PHYSICS_DOGMODEL_H
#define DOG_CONTROL_PHYSICS_DOGMODEL_H

#include "dog_control/utils/ClassDeclare.h"

#include "FloatingBaseModel.h"
#include "dog_control/message/JointState.h"
#include "dog_control/message/LegName.h"
#include "dog_control/utils/ParamDict.h"

#include <boost/weak_ptr.hpp>

namespace dog_control
{

namespace physics
{

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

    /**
     * @brief ComputeLocalPos
     * Compute foot local position according to given joint positions
     * @param leg_name          the leg to compute
     * @param joint_pos         joint positions
     * @return
     */
    Eigen::Vector3d ComputeLocalPos(
            LegName leg_name, const Eigen::Vector3d &joint_pos) const;

    /**
     * @brief InverseKinematics
     * Computes desired joint positions of leg joints to achieve
     * the target foot position (in local frame).
     * @param leg_name          the leg to compute
     * @param foot_local_pos    foot position in torso's frame
     * @param knee_out          if the knee curves outwards
     * @param hip_out           if the thigh is at side of the torso
     * @return                  desired joint position
     */
    Eigen::Vector3d InverseKinematics(LegName leg_name,
                                      const Eigen::Vector3d &foot_local_pos,
                                      bool knee_out, bool hip_out) const;

    /**
     * @brief ComputeJacobian
     * Compute joint jacobian in torso's frame (local frame),
     * with the given joint pos.
     * @param leg_name          the leg to compute
     * @param joint_pos         joint positions
     * @return                  jacobian matrix
     */
    Eigen::Matrix3d ComputeJacobian(
            LegName leg_name, const Eigen::Vector3d &joint_pos) const;

    /**
     * @brief ComputeDJacobian
     * Compute the jacobian's derivative with respect to time,
     * with the given joint states.
     * @param leg_name          the leg to compute
     * @param joint_pos         joint positions
     * @param joint_vel         joint velocities
     * @return                  dJ / dt
     */
    Eigen::Matrix3d ComputeDJacobian(
            LegName leg_name, const Eigen::Vector3d &joint_pos,
            const Eigen::Vector3d &joint_vel) const;

    /**
     * @brief LocalJacob
     * Compute joint jacobian in torso's frame (local frame).
     * @param leg_name          the leg to compute
     * @return                  matrix J that satisfies dpos = J * dq
     */
    Eigen::Matrix3d LocalJacob(LegName leg_name) const;

    /**
     * @brief LocalDJacob
     * Compute joint jacobian derivative in torso's frame (local frame).
     * @param leg_name          the leg to compute
     * @return                  dJ / dt
     */
    Eigen::Matrix3d LocalDJacob(LegName leg_name) const;

//    using spatial::FloatingBaseModel::SetJointMotionState;
    using spatial::FloatingBaseModel::ForwardKinematics;
    using spatial::FloatingBaseModel::MassMatrix;
    using spatial::FloatingBaseModel::BiasForces;

    message::FloatingBaseState TorsoState() const;

    Eigen::VectorXd Vq() const;

    /**
     * @brief FootPos
     * Compute global foot position.
     * @param leg_name          the leg to compute
     * @return                  global foot position
     */
    Eigen::Vector3d FootPos(LegName leg_name) const;

    /**
     * @brief FootVel
     * Compute global foot velocity.
     * @param leg_name          the leg to compute
     * @return                  global foot velocity
     */
    Eigen::Vector3d FootVel(LegName leg_name) const;

    /**
     * @brief FootContact
     * @return                  current foot contact state
     */
    std::array<bool, 4> FootContact() const;

    /**
     * @brief FullJacob
     * Get the complete jacobian of a foot.
     * The full jacobian matrix has a size of 3 x 18.
     * @param leg_name          the leg to compute
     * @return                  jacobian matrix.
     */
    FullJacobMat FullJacob(LegName leg_name) const;

    Eigen::Vector3d VJDotVq(LegName leg_name) const;

    /**
     * @brief Friction
     * Compute joints' friction (with their velocity given)
     * @return      friction. When joint velocity > 0, friction < 0.
     */
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

    // foot pos and vel
    std::array<Eigen::Vector3d, 4> foot_pos_;
    std::array<Eigen::Vector3d, 4> foot_vel_;
    std::array<bool, 4> foot_contact_;

    boost::weak_ptr<hardware::HardwareBase> hw_ptr_;
    boost::weak_ptr<estimator::EstimatorBase> est_ptr_;
};

} /* physics */

} /* dog_control */

#endif /* DOG_CONTROL_PHYSICS_DOGMODEL_H */
