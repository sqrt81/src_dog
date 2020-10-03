#ifndef DOG_CONTROL_PHYSICS_FLOATINGBASEMODELPROPERTIES_H
#define DOG_CONTROL_PHYSICS_FLOATINGBASEMODELPROPERTIES_H

#include <Eigen/Eigen>

namespace dog_control
{

namespace physics
{

namespace spatial
{

/**
 * Defines spatial vectors (SVec) and spatial matrix (SMat) here.
 * There are two types of spatial vectors: motion vector and force vector.
 * They are different in frame transform.
 * motion vector is a way of representing rigid body motion,
 * while force vector can be used to represent forces and torque
 * applied on a rigid body.
 * Such way of representation and many of its applications are discussed
 * in "Rigid Body Dynamics Algorithms" by Featherstone.
 * (download from
 * https://www.springer.com/us/book/9780387743141 on MIT internet)
 */
using SVec = Eigen::Matrix<double, 6, 1>;
using SMat = Eigen::Matrix<double, 6, 6>;

enum JointType
{
    floating = 0,
    revolute = 1
};

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

    // Actuated joints usually have a motor with it,
    // and a motor usually contains a rotor whose movement cannot be ignored.
    // Similar to MIT's approach, we add description for the rotor so that
    // the dynamics algorithm could deal with it.
    // For floating joints, these properties are ignored.
    SMat rotor_inertia;
    SMat rotor_X_parent; // rotor's transform in parent's frame
    double gear_ratio; // inverse of reduction ratio.
};

struct EndEffectorInfo
{
    Eigen::Vector3d ee_local_pos;
    int ee_link_id;
};

} /* spatial */

} /* physics */

} /* dog_control */

#endif /* DOG_CONTROL_PHYSICS_FLOATINGBASEMODELPROPERTIES_H */
