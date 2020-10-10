#include "dog_control/physics/DogModel.h"

#include "dog_control/physics/SpatialToolbox.h"
#include "dog_control/utils/Math.h"

#include "dog_control/utils/MiniLog.h"

namespace dog_control
{

namespace
{

inline double ReadParOrDie(utils::ParamDictCRef dict,
                           const std::string &param)
{
    const utils::ParamDict::const_iterator iter = dict.find(param);

    CHECK(iter != dict.end()) << "Param \"" << param << "\" not found.";

    return iter->second;
}

} /* anonymous */

namespace physics
{

using namespace spatial;

void DogModel::Initialize(utils::ParamDictCRef dict)
{
    hip_pos_x_ = ReadParOrDie(dict, PARAM_WITH_NS(hip_pos_x, physics));
    hip_pos_y_ = ReadParOrDie(dict, PARAM_WITH_NS(hip_pos_y, physics));
    hip_len_y_ = ReadParOrDie(dict, PARAM_WITH_NS(hip_len_y, physics));
    thigh_offset_z_
            = ReadParOrDie(dict, PARAM_WITH_NS(thigh_offset_z, physics));
    shin_offset_z_
            = ReadParOrDie(dict, PARAM_WITH_NS(shin_offset_z, physics));

    friction_ = ReadParOrDie(dict, PARAM_WITH_NS(friction, motor));
    damping_ = ReadParOrDie(dict, PARAM_WITH_NS(damping, motor));
    vel_deadband_ = ReadParOrDie(dict, PARAM_WITH_NS(vel_deadband, motor));
    const double gear_ratio
            = ReadParOrDie(dict, PARAM_WITH_NS(gear_ratio, motor));
    const Eigen::Vector3d ee_local_pos(0, 0, shin_offset_z_);

    node_description_.resize(13);
    parent_.resize(13);
    ee_info_.resize(4);

    Eigen::Matrix3d inertial = Eigen::Matrix3d::Zero();
    Eigen::Vector3d com = Eigen::Vector3d::Zero();
    Eigen::Vector3d tf_parent = Eigen::Vector3d::Zero();
    double mass;

    mass = ReadParOrDie(dict, PARAM_WITH_NS(torso_mass, physics));
    inertial(0, 0) = ReadParOrDie(dict, PARAM_WITH_NS(I0xx, physics));
    inertial(1, 1) = ReadParOrDie(dict, PARAM_WITH_NS(I0yy, physics));
    inertial(2, 2) = ReadParOrDie(dict, PARAM_WITH_NS(I0zz, physics));
    node_description_[0].inertia = BuildInertia(mass, com, inertial);
    node_description_[0].joint_type = floating;
    parent_[0] = - 1;

    // node of the hips
    inertial(0, 0) = ReadParOrDie(dict, PARAM_WITH_NS(I1xx, physics));
    inertial(1, 1) = ReadParOrDie(dict, PARAM_WITH_NS(I1yy, physics));
    inertial(2, 2) = ReadParOrDie(dict, PARAM_WITH_NS(I1zz, physics));
    com = Eigen::Vector3d(
                ReadParOrDie(dict, PARAM_WITH_NS(com_hip_x, physics)),
                ReadParOrDie(dict, PARAM_WITH_NS(com_hip_y, physics)),
                0);
    mass = ReadParOrDie(dict, PARAM_WITH_NS(hip_mass, physics));
    tf_parent = Eigen::Vector3d(hip_pos_x_, hip_pos_y_, 0);

    for (int i = 0; i < 4; i++)
    {
        NodeDescription& nd = node_description_[i * 3 + 1];
        const Eigen::Vector3d _com(com(0) * (i < 2      ? 1 : - 1),
                                   com(1) * (i % 2 == 0 ? 1 : - 1),
                                   com(2));
        const Eigen::Vector3d _tf(tf_parent(0) * (i < 2      ? 1 : - 1),
                                  tf_parent(1) * (i % 2 == 0 ? 1 : - 1),
                                  tf_parent(2));
        nd.inertia = BuildInertia(mass, _com, inertial);
        nd.joint_type = revolute;
        nd.joint_axis << (i % 2 == 0 ? 1 : -1), 0, 0, 0, 0, 0;
        nd.X_parent = BuildTransform(_tf, Eigen::Quaterniond(1, 0, 0, 0));
        nd.rotor_inertia = SMat::Zero(); // rotor is not modeled yet!
        nd.rotor_X_parent = nd.X_parent;
        nd.gear_ratio = gear_ratio;

        parent_[i * 3 + 1] = 0;
    }

    // thigh
    inertial(0, 0) = ReadParOrDie(dict, PARAM_WITH_NS(I2xx, physics));
    inertial(1, 1) = ReadParOrDie(dict, PARAM_WITH_NS(I2yy, physics));
    const double I2yz = ReadParOrDie(dict, PARAM_WITH_NS(I2yz, physics));
    inertial(2, 2) = ReadParOrDie(dict, PARAM_WITH_NS(I2zz, physics));
    com = Eigen::Vector3d(0,
                ReadParOrDie(dict, PARAM_WITH_NS(com_thigh_y, physics)),
                ReadParOrDie(dict, PARAM_WITH_NS(com_thigh_z, physics)));
    mass = ReadParOrDie(dict, PARAM_WITH_NS(thigh_mass, physics));
    tf_parent = Eigen::Vector3d(
                ReadParOrDie(dict, PARAM_WITH_NS(hip_len_x, physics)),
                hip_len_y_,
                0);

    for (int i = 0; i < 4; i++)
    {
        NodeDescription& nd = node_description_[i * 3 + 2];
        const Eigen::Vector3d _com(com(0) * (i < 2      ? 1 : - 1),
                                   com(1) * (i % 2 == 0 ? 1 : - 1),
                                   com(2));
        const Eigen::Vector3d _tf(tf_parent(0) * (i < 2      ? 1 : - 1),
                                  tf_parent(1) * (i % 2 == 0 ? 1 : - 1),
                                  tf_parent(2));
        inertial(1, 2) = i % 2 == 0 ? I2yz : - I2yz;
        inertial(2, 1) = inertial(1, 2);
        nd.inertia = BuildInertia(mass, _com, inertial);
        nd.joint_type = revolute;
        nd.joint_axis << 0, (i < 2 ? 1 : -1), 0, 0, 0, 0;
        nd.X_parent = BuildTransform(_tf, Eigen::Quaterniond(1, 0, 0, 0));
        nd.rotor_inertia = SMat::Zero(); // rotor is not modeled yet!
        nd.rotor_X_parent = nd.X_parent;
        nd.gear_ratio = gear_ratio;

        parent_[i * 3 + 2] = i * 3 + 1;
    }

    // shin
    inertial(0, 0) = ReadParOrDie(dict, PARAM_WITH_NS(I3xx, physics));
    inertial(1, 1) = ReadParOrDie(dict, PARAM_WITH_NS(I3yy, physics));
    inertial(1, 2) = 0;
    inertial(2, 1) = 0;
    inertial(2, 2) = ReadParOrDie(dict, PARAM_WITH_NS(I3zz, physics));
    com = Eigen::Vector3d(0, 0,
                ReadParOrDie(dict, PARAM_WITH_NS(com_shin_z, physics)));
    mass = ReadParOrDie(dict, PARAM_WITH_NS(shin_mass, physics));
    tf_parent = Eigen::Vector3d(0, 0, thigh_offset_z_);

    for (int i = 0; i < 4; i++)
    {
        NodeDescription& nd = node_description_[i * 3 + 3];
        const Eigen::Vector3d _com(com(0) * (i < 2      ? 1 : - 1),
                                   com(1) * (i % 2 == 0 ? 1 : - 1),
                                   com(2));
        const Eigen::Vector3d _tf(tf_parent(0) * (i < 2      ? 1 : - 1),
                                  tf_parent(1) * (i % 2 == 0 ? 1 : - 1),
                                  tf_parent(2));
        nd.inertia = BuildInertia(mass, _com, inertial);
        nd.joint_type = revolute;
        nd.joint_axis << 0, (i < 2 ? 1 : -1), 0, 0, 0, 0;
        nd.X_parent = BuildTransform(_tf, Eigen::Quaterniond(1, 0, 0, 0));
        nd.rotor_inertia = SMat::Zero(); // rotor is not modeled yet!
        nd.rotor_X_parent = nd.X_parent;
        nd.gear_ratio = gear_ratio;

        parent_[i * 3 + 3] = i * 3 + 2;

        const Eigen::Vector3d _ee_local_pos(
                    ee_local_pos(0) * (i < 2      ? 1 : - 1),
                    ee_local_pos(1) * (i % 2 == 0 ? 1 : - 1),
                    ee_local_pos(2));
        ee_info_[i].ee_link_id = i * 3 + 3;
        ee_info_[i].ee_local_pos = _ee_local_pos;
    }

    // for inverse kinematics convenience
    hip_pos_x_ += ReadParOrDie(dict, PARAM_WITH_NS(hip_len_x, physics));
}

void DogModel::ConnectHardware(boost::shared_ptr<hardware::HardwareBase> hw)
{
    hw_ptr_ = hw;
}

void DogModel::ConnectEstimator(
        boost::shared_ptr<estimator::EstimatorBase> est)
{
    est_ptr_ = est;
}

Eigen::Vector3d DogModel::InverseKinematics(LegName leg_name,
                                 const Eigen::Vector3d &foot_local_pos,
                                 bool knee_out, bool hip_out) const
{
    const double x_h
            = foot_local_pos.x() * (leg_name < 2      ? 1 : - 1) - hip_pos_x_;
    const double y_h
            = foot_local_pos.y() * (leg_name % 2 == 0 ? 1 : - 1) - hip_pos_y_;
    const double z_t = foot_local_pos.z();

    const double cos_11 = hip_len_y_ / sqrt(z_t * z_t + y_h * y_h);

    const double angle_11 = acos(cos_11 < 1. ? cos_11 : 1.);
    const double angle_12 = atan2(z_t, y_h);

    const double q1 = hip_out ? (angle_11 + angle_12) : (angle_12 - angle_11);

    const double w_h = y_h * sin(q1) - z_t * cos(q1);
    const double leg_len_2 = x_h * x_h + w_h * w_h;
    const double cos_3
            = (leg_len_2 - thigh_offset_z_ * thigh_offset_z_
               - shin_offset_z_ * shin_offset_z_)
              / (2 * thigh_offset_z_ * shin_offset_z_);
    const double angle_3 = acos(utils::clamp(cos_3, - 1., 1.));

    const double q3 = knee_out ? angle_3 : (- angle_3);
    const double sin_22 = shin_offset_z_ * sin(q3) / sqrt(leg_len_2);

    const double q2 = - atan2(x_h, w_h)
            + asin(utils::clamp(sin_22, - 1., 1.));

    return Eigen::Vector3d(q1, q2, q3);
}

Eigen::Matrix3d DogModel::LocalJacob(LegName leg_name) const
{
    VALID_LEGNAME(leg_name);

    const double c1 = cos(js_.q[leg_name * 3]);
    const double s1 = sin(js_.q[leg_name * 3]);
    const double c2 = cos(js_.q[leg_name * 3 + 1]);
    const double s2 = sin(js_.q[leg_name * 3 + 1]);
    const double c23 = cos(js_.q[leg_name * 3 + 1] + js_.q[leg_name * 3 + 2]);
    const double s23 = sin(js_.q[leg_name * 3 + 1] + js_.q[leg_name * 3 + 2]);

    const double shin_z = thigh_offset_z_ * c2 + shin_offset_z_ * c23;
    const double shin_x = thigh_offset_z_ * s2 + shin_offset_z_ * s23;

    const int sign_x = leg_name < 2      ? 1 : - 1;
    const int sign_y = leg_name % 2 == 0 ? 1 : - 1;

    Eigen::Matrix3d jacob;

    jacob(0, 0) = 0.;
    jacob(0, 1) = sign_x * shin_z;
    jacob(0, 2) = sign_x * shin_offset_z_ * c23;
    jacob(1, 0) = sign_y * (- shin_z * c1 - hip_len_y_ * s1);
    jacob(1, 1) = sign_y * shin_x * s1;
    jacob(1, 2) = sign_y * shin_offset_z_ * s1 * s23;
    jacob(2, 0) = - shin_z * s1 + hip_len_y_ * c1;
    jacob(2, 1) = - shin_x * c1;
    jacob(2, 2) = - shin_offset_z_ * c1 * s23;

    return jacob;
}

Eigen::Matrix3d DogModel::ComputeJacobian(
        LegName leg_name, const Eigen::Vector3d &joint_local_pos) const
{
    VALID_LEGNAME(leg_name);

    const double c1 = cos(joint_local_pos(0));
    const double s1 = sin(joint_local_pos(0));
    const double c2 = cos(joint_local_pos(1));
    const double s2 = sin(joint_local_pos(1));
    const double c23 = cos(joint_local_pos(1) + joint_local_pos(2));
    const double s23 = sin(joint_local_pos(1) + joint_local_pos(2));

    const double shin_z = thigh_offset_z_ * c2 + shin_offset_z_ * c23;
    const double shin_x = thigh_offset_z_ * s2 + shin_offset_z_ * s23;

    const int sign_x = leg_name < 2      ? 1 : - 1;
    const int sign_y = leg_name % 2 == 0 ? 1 : - 1;

    Eigen::Matrix3d jacob;

    jacob(0, 0) = 0.;
    jacob(0, 1) = sign_x * shin_z;
    jacob(0, 2) = sign_x * shin_offset_z_ * c23;
    jacob(1, 0) = sign_y * (- shin_z * c1 - hip_len_y_ * s1);
    jacob(1, 1) = sign_y * shin_x * s1;
    jacob(1, 2) = sign_y * shin_offset_z_ * s1 * s23;
    jacob(2, 0) = - shin_z * s1 + hip_len_y_ * c1;
    jacob(2, 1) = - shin_x * c1;
    jacob(2, 2) = - shin_offset_z_ * c1 * s23;

    return jacob;
}

message::FloatingBaseState DogModel::TorsoState() const
{
    return js_.base;
}

Eigen::VectorXd DogModel::Vq() const
{
    Eigen::VectorXd vq(18);
    vq.segment<3>(0) = js_.base.linear_vel;
    vq.segment<3>(3) = js_.base.rot_vel;

    for (int i = 0; i < 12; i++)
        vq(i + 6) = js_.dq[i];

    return vq;
}

Eigen::Vector3d DogModel::FootPos(LegName leg_name)
{
    VALID_LEGNAME(leg_name);

    if (!kinematics_updated_)
        ForwardKinematics();

    return EEPos(leg_name);
}

Eigen::Vector3d DogModel::FootVel(LegName leg_name)
{
    VALID_LEGNAME(leg_name);

    if (!kinematics_updated_)
        ForwardKinematics();

    return EEVel(leg_name);
}

DogModel::FullJacobMat DogModel::FullJacob(LegName leg_name)
{
    VALID_LEGNAME(leg_name);

    if (!kinematics_updated_)
        ForwardKinematics();

    FullJacobMat jacob = FullJacobMat::Zero();
    const Eigen::Matrix3d foot_pos = ToLowerMatrix(FootPos(leg_name));

//    for (int i = ee_info_[leg_name].ee_link_id; i != 0; i = parent_[i])
//    {
//        const NodeDescription& nd = node_description_[i];

//        switch(nd.joint_type)
//        {
//        case revolute:
//            // manually calculate transform from link i to base.
//            // The transform matrix is split into four parts.
//            // Only the top-left and bottom-left parts are concerned
//            // since nd.joint_axis's last 3 elements are zero.
//        {
//            const Eigen::Matrix3d rot_t
//                    = X0_[i].topLeftCorner<3, 3>().transpose();
//            jacob.col(i + 5)
//                    = (- rot_t * X0_[i].bottomLeftCorner<3, 3>() - foot_pos)
//                    * rot_t * nd.joint_axis.topRows<3>();
//        }
//            break;
//        default:
//            assert(false);
//        }
//    }

    // The method above is the general way of computing jacobian matrix.
    // However, in case of this robot model, we can use some shortcuts.
    jacob.middleCols<3>(leg_name * 3 + 6)
            = X0_[0].topLeftCorner<3, 3>().transpose() * LocalJacob(leg_name);

    {
        const Eigen::Matrix3d rot_t
                = X0_[0].topLeftCorner<3, 3>().transpose();
        jacob.leftCols<3>()
                = (- rot_t * X0_[0].bottomLeftCorner<3, 3>() - foot_pos)
                * rot_t;
        jacob.middleCols<3>(3) = rot_t;
    }

    return jacob;
}

Eigen::Vector3d DogModel::VJDotVq(LegName leg_name)
{
    VALID_LEGNAME(leg_name);

    return LinearVel(vJ_vq_[leg_name * 3 + 1],
                     ee_info_[leg_name].ee_local_pos);
}

DogModel::JointForce DogModel::Friction() const
{
    JointForce friction;

    for (int i = 0; i < 12; i++)
    {
        const double vel = js_.dq[i];

        if (vel > vel_deadband_)
            friction(i) = - vel * damping_ - friction_;
        else if (vel < - vel_deadband_)
            friction(i) = - vel * damping_ + friction_;
        else
            friction(i) = - vel * damping_;
    }

    return friction;
}

void DogModel::Update()
{
    boost::shared_ptr<hardware::HardwareBase> hw = hw_ptr_.lock();
    boost::shared_ptr<estimator::EstimatorBase> est = est_ptr_.lock();
    assert(hw);
    assert(est);

    message::StampedJointState js = hw->GetJointState();

    FloatingBaseModel::FBJS js_fb;

    message::EstimatorResult res;
    est->WriteResult(res);

    js_fb.base.rot = res.orientation;
    js_fb.base.trans = res.position;
    js_fb.base.linear_vel = res.linear_vel;
    js_fb.base.rot_vel = res.rot_vel;

    js_fb.q.resize(12);
    js_fb.dq.resize(12);

    for (int i = 0; i < 12; i++)
    {
        js_fb.q[i] = js.joint_state[i].pos;
        js_fb.dq[i] = js.joint_state[i].vel;
    }

    SetJointMotionState(js_fb);
}

} /* physics */

} /* dog_control */
