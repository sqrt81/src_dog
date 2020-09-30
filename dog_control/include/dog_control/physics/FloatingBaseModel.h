#ifndef DOG_CONTROL_PHYSICS_FLOATINGBASEMODEL_H
#define DOG_CONTROL_PHYSICS_FLOATINGBASEMODEL_H

#include <vector>
#include <Eigen/Eigen>

#include "SpatialToolbox.h"

namespace dog_control
{

namespace physics
{

namespace spatial
{

/**
 * @brief NodeDescription
 * A link, together with the joint connecting it with its parent,
 * are considered as a "node".
 */
struct NodeDescription
{
    SMat inertia; // link's spatial inertia measured in joint frame

    // describing the joint connecting this link to its parent
    JointType joint_type;
    SVec joint_axis;
    // joint vel = joint_axis * vq

    // Since the joint motion space and link inertia are measured
    // in the same frame, the transform is always identity.
    // However, the joint's position in parent link needs to be defined
    SMat X_parent;
};

/**
 * @brief JointState
 * Position and velocity for all the joints
 */
struct FloatingBaseJointState
{
    Eigen::Vector3d base_trans;
    Eigen::Quaterniond base_rot;

    // note that base velocity is measured in base frame,
    // not in fixed base (global) frame
    Eigen::Vector3d base_linear_vel;
    Eigen::Vector3d base_rot_vel;

    std::vector<double> q;
    std::vector<double> dq;
};

struct EndEffectorInfo
{
    Eigen::Vector3d ee_local_pos;
    int ee_link_id;
};

/**
 * @brief The FloatingBaseModel class
 * Describes a rigid-body tree with a floating base.
 * The described rigid-body tree should have all links
 * connected by 1-dof joints except for connection to
 * the fixed base.
 */
class FloatingBaseModel
{
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
    void SetJointMotionState(const FloatingBaseJointState& stat);

    /**
     * @brief ForwardKinematics
     * Compute forward kinematics of the body (if an update is needed)
     */
    void ForwardKinematics();

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

private:
    std::vector<NodeDescription> node_description_;
    std::vector<int> parent_;
    std::vector<EndEffectorInfo> ee_info_;

    bool kinematics_updated_;

    FloatingBaseJointState js_;

    // Two points need to be made clear:
    // First, a mat X transform from base to link i means that,
    // for any motion vector represented in base frame v_b,
    // the same vector represented in link i frame v_i is equal to
    // X * v_b.
    // Second, when I say in XXX frame, I actually means in a
    // coordinate system which stays still with fix frame
    // while coincide with XXX frame's coordinate system at this moment.
    std::vector<SVec> v_; // spatial vel for link i (in its own frame)
    std::vector<SMat> X0_; // transform matrix from fixed base to link i
};

} /* spatial */

} /* physics */

} /* dog_control */

#endif /* DOG_CONTROL_PHYSICS_FLOATINGBASEMODEL_H */
