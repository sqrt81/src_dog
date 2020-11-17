#ifndef DOG_CONTROL_PHYSICS_FLOATINGBASEMODEL_H
#define DOG_CONTROL_PHYSICS_FLOATINGBASEMODEL_H

#include <vector>
#include <Eigen/Eigen>

#include "dog_control/message/ModelJointState.h"
#include "SpatialProperties.h"

namespace dog_control
{

namespace physics
{

namespace spatial
{

/**
 * @brief The FloatingBaseModel class
 * Describes a rigid-body tree with a floating base.
 * The described rigid-body tree should have all links
 * connected by 1-dof joints except for connection to
 * the fixed base.
 */
class FloatingBaseModel
{
protected:
    using FBJS = message::FloatingBaseJointState;
    using FBJSCRef = message::FBJSCRef;
    using FBJSDCRef = message::FBJSDCRef;

public:
    FloatingBaseModel() = default;

    /**
     * @brief AddLink
     * Add a link (and joint connecting its parent) to the model
     * @param node      description of the link and joint
     * @param parent    parent link id
     * @return          added link's id, begin from 1
     *  (since 0 is assigned to fixed base)
     */
    int AddLink(const NodeDescription& node, int parent);

    /**
     * @brief AddEndEffector
     * Add an end effector, whose vel and pos can be easily assessed then.
     * @param link_id       the link this end effector attached to
     * @param ee_local_pos  local position of the end effector
     * @return              end effector's id, begin from 0
     */
    int AddEndEffector(int link_id, const Eigen::Vector3d& ee_local_pos);

    /**
     * @brief SetJointMotionState
     * Update position and velocity of each joints.
     * @param stat      joint states
     */
    void SetJointMotionState(FBJSCRef stat);

    /**
     * @brief ForwardKinematics
     * Compute forward kinematics of the body (if an update is needed).
     * Transform, velocity and Coriolis acceleration
     * of each link and rotor are computed in this method.
     */
    void ForwardKinematics();

    /**
     * @brief BiasForces
     * Compute spatial forces (on torso) and joint torque needed
     * for keeping current joint velocity (and torso velocity).
     * Note that gravity is excluded, since it can be modeled as
     * floating base's linear acceleration.
     * @return          vector C with (6 + joint_cnt) elements
     */
    Eigen::VectorXd BiasForces();

    /**
     * @brief BaseForceJacobian
     * Compute base bias force's jacobian w.r.t. base spatial motion.
     * @return          6 * 6 jacobian matrix
     */
    SMat BaseForceJacobian();

    /**
     * @brief MassMatrix
     * Mass matrix H shows the relationship between
     * (torso & joint) acceleration and joint torque.
     * Specifically speaking, H(i, j) is the required force / torque
     * for joint i,
     * if we want j to have unit acceleration while keeping
     * other joints moving with no acceleration.
     * Note that the matrix H is symmetric.
     * @return          (6 + joint_cnt) x (6 + joint_cnt) mass matrix H
     */
    Eigen::MatrixXd MassMatrix();

    /**
     * @brief EEPos
     * Compute global position of the specified end effector
     * @param ee_id     end effector id
     * @return          end effector position in fixed base frame
     */
    Eigen::Vector3d EEPos(int ee_id) const;

    /**
     * @brief EEVel
     * Compute global velocity of the specified end effector
     * @param ee_id     end effector id
     * @return          end effector velocity in fixed base frame
     */
    Eigen::Vector3d EEVel(int ee_id) const;

    /**
     * @brief DemoInfo
     * Print info for debugging.
     */
    void DemoInfo();

protected:
    std::vector<NodeDescription> node_description_;
    std::vector<int> parent_;
    std::vector<EndEffectorInfo> ee_info_;

    bool kinematics_updated_;
    bool bias_force_updated_;
    bool mass_matrix_updated_;
    bool base_bias_jacob_updated_;

    FBJS js_;

//    SVec a_base_; // fixed base acceleration. Used as gravity.

    // Two points need to be made clear:
    // First, a mat X transform from base to link i means that,
    // for any motion vector represented in base frame v_b,
    // the same vector represented in link i frame v_i is equal to
    // X * v_b.
    // Second, when I say in XXX frame, I actually means in a
    // coordinate system which stays still with fix frame
    // while coincide with XXX frame's coordinate system at this moment.
    std::vector<SVec> v_; // spatial vel for link i (in its own frame)
    std::vector<SMat> X_parent_; // transform matrix from parent to this link
    std::vector<SMat> X0_; // transform matrix from fixed base to link i
    std::vector<SMat> X0_inv_; // inverse of X0_
    std::vector<SVec> a_C_; // Coriolis acceleration, caused by rotation
    std::vector<SVec> vJ_vq_; // dJ/dt * vq, used to calculated acceleration.

    // Same properties for rotors.
    std::vector<SVec> v_rot_;
    std::vector<SMat> X_parent_rot_; // transform from parent link to rotor
    std::vector<SMat> X0_rot_;
    std::vector<SVec> a_C_rot_;

    // joint forces needed for zero q acceleration.
    Eigen::VectorXd compensate_;
    Eigen::MatrixXd mass_matrix_;
    SMat base_bias_jacob_;
};

} /* spatial */

} /* physics */

} /* dog_control */

#endif /* DOG_CONTROL_PHYSICS_FLOATINGBASEMODEL_H */
