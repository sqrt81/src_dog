#ifndef DOG_CONTROL_PHYSICS_DOGPHYSICS_H
#define DOG_CONTROL_PHYSICS_DOGPHYSICS_H

#include "dog_control/utils/ParamDict.h"
#include "JointState.h"

#include <Eigen/Eigen>

namespace dog_control
{

namespace physics
{

/**
 * @brief The DogPhysics class
 * This class computes all values relative to
 * robot kinematics and dynamics.
 * All values are calculated based on
 * configuration of front-left leg
 */
class DogPhysics
{
public:
    DogPhysics() = default;

    void Initialize(utils::ParamDictCRef dict);

    /**
     * @brief ForwardKinematics
     * Given joint positions, this function
     * computes the FL foot position relative to torso.
     * @param   joint_pos joint positions of front-left leg.
     * @return  front-left foot position.
     */
    Eigen::Vector3d ForwardKinematics(JointState3CRef joint_pos) const;

    /**
     * @brief ComputeJacobian
     * Computes joint jacobian.
     * @param joint_pos joint position of front-left leg.
     * @return jacobian matrix J that satisfies dpos = J * dq.
     */
    Eigen::Matrix3d ComputeJacobian(JointState3CRef joint_pos) const;

    /**
     * @brief InverseKinematics
     * Computes desired joint positions of FL leg
     * with the target foot position.
     * @param foot_pos  target foot position.
     * @param joint_pos desired joint position.
     * @param knee_out  if the knee curves outwards
     * @param hip_out   if the thigh is at side of the torso
     */
    void InverseKinematics(const Eigen::Vector3d &foot_pos,
                           JointState3 &joint_pos,
                           bool knee_out, bool hip_out) const;

    /**
     * @brief BuildDynamicsMatrixs
     * Given joint pos and vel, build dynamics matrix H, C, G
     * that satisfies:
     * tau = H * acc + C * vel + G * gravity + friction.
     * Where acc, vel are joint accelerations and velocities
     * and gravity is gravitational acceleration in leg frame.
     * @param joint_stat joint positions and velocities of FL leg
     */
    void BuildDynamicsMatrixs(JointState3CRef joint_stat,
                              Eigen::Matrix3d &H,
                              Eigen::Matrix3d &C,
                              Eigen::Matrix3d &G) const;

    /**
     * @brief ComputeTorque
     * Compute torque according to formula:
     * tau = H * acc + C * vel + G * gravity
     * @param joint_stat pos, vel and acc of joints
     * @param gravity gravity in front-left leg frame
     * @return joint torque without friction compensation
     */
    Eigen::Vector3d ComputeTorque(JointState3CRef joint_stat,
                                  const Eigen::Vector3d &gravity) const;

private:
    /**
     * @brief ComputeFriction Compute joint friction given its velocity
     * @param joint_stat states which contains velocity of joint(s)
     * @return friction. When joint velocity > 0, friction < 0.
     */
    Eigen::Vector3d ComputeFriction(JointState3CRef joint_stat) const;

    // length
    double hip_pos_x_; // hip_pos_x_ = hip_pos_x + hip_len_x
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

    // motor friction
    double friction_;
    double damping_;
    double vel_deadband_;
};

} /* physics */

} /* dog_control */

#endif /* DOG_CONTROL_PHYSICS_DOGPHYSICS_H */
