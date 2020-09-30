#include "dog_control/physics/DogPhysics.h"

#include <cassert>
#include <cmath>

using std::cos;
using std::sin;
using std::acos;
using std::atan2;

#define READ_PARAM_OR_DIE(dict, param, ns) \
    do \
    { \
        const utils::ParamDict::const_iterator iter \
            = dict.find(#ns "/" #param); \
        assert(iter != dict.end()); \
        param ## _ = iter->second; \
    } \
    while(false)


namespace dog_control
{

namespace physics
{

void DogPhysics::Initialize(utils::ParamDictCRef dict)
{
    double hip_len_x_;

    READ_PARAM_OR_DIE(dict, hip_pos_x, physics);
    READ_PARAM_OR_DIE(dict, hip_pos_y, physics);
    READ_PARAM_OR_DIE(dict, hip_len_x, physics);
    READ_PARAM_OR_DIE(dict, hip_len_y, physics);
    hip_pos_x_ += hip_len_x_;
    READ_PARAM_OR_DIE(dict, thigh_offset_z, physics);
    READ_PARAM_OR_DIE(dict, shin_offset_z, physics);

    READ_PARAM_OR_DIE(dict, com_hip_y, physics);
    READ_PARAM_OR_DIE(dict, com_thigh_y, physics);
    READ_PARAM_OR_DIE(dict, com_thigh_z, physics);
    READ_PARAM_OR_DIE(dict, com_shin_z, physics);

    READ_PARAM_OR_DIE(dict, hip_mass, physics);
    READ_PARAM_OR_DIE(dict, thigh_mass, physics);
    READ_PARAM_OR_DIE(dict, shin_mass, physics);

    READ_PARAM_OR_DIE(dict, I1xx, physics);
    READ_PARAM_OR_DIE(dict, I2xx, physics);
    READ_PARAM_OR_DIE(dict, I2yy, physics);
    READ_PARAM_OR_DIE(dict, I2yz, physics);
    READ_PARAM_OR_DIE(dict, I2zz, physics);
    READ_PARAM_OR_DIE(dict, I3xx, physics);
    READ_PARAM_OR_DIE(dict, I3yy, physics);
    READ_PARAM_OR_DIE(dict, I3zz, physics);

    READ_PARAM_OR_DIE(dict, friction, motor);
    READ_PARAM_OR_DIE(dict, damping, motor);
    READ_PARAM_OR_DIE(dict, vel_deadband, motor);
}

Eigen::Vector3d DogPhysics::ForwardKinematics(JointState3CRef joint_pos) const
{
    const double c1 = cos(joint_pos[0].pos);
    const double s1 = sin(joint_pos[0].pos);
    const double c2 = cos(joint_pos[1].pos);
    const double s2 = sin(joint_pos[1].pos);
    const double c23 = cos(joint_pos[1].pos + joint_pos[2].pos);
    const double s23 = sin(joint_pos[1].pos + joint_pos[2].pos);

    const double shin_z = thigh_offset_z_ * c2 + shin_offset_z_ * c23;
    const double shin_x = thigh_offset_z_ * s2 + shin_offset_z_ * s23;

    return Eigen::Vector3d(
                hip_pos_x_ + shin_x,
                hip_len_y_ * c1 - shin_z * s1 + hip_pos_y_,
                hip_len_y_ * s1 + shin_z * c1);
}

void DogPhysics::InverseKinematics(const Eigen::Vector3d &foot_pos,
                                   JointState3 &joint_pos,
                                   bool knee_out, bool hip_out) const
{
    const double x_h = foot_pos.x() - hip_pos_x_;
    const double y_h = foot_pos.y() - hip_pos_y_;
    const double z_t = foot_pos.z();

    const double angle_11 = acos(hip_len_y_ / sqrt(z_t * z_t + y_h * y_h));
    const double angle_12 = atan2(z_t, y_h);

    const double q1 = hip_out ? (angle_11 + angle_12) : (angle_12 - angle_11);

    const double w_h = y_h * sin(q1) - z_t * cos(q1);
    const double leg_len_2 = x_h * x_h + w_h * w_h;
    const double angle_3
            = acos((leg_len_2 - thigh_offset_z_ * thigh_offset_z_
                    - shin_offset_z_ * shin_offset_z_)
                   / (2 * thigh_offset_z_ * shin_offset_z_));

    const double q3 = knee_out ? angle_3 : (- angle_3);

    const double q2 = - atan2(x_h, w_h)
            + asin(shin_offset_z_ * sin(q3) / sqrt(leg_len_2));

    joint_pos[0].pos = q1;
    joint_pos[1].pos = q2;
    joint_pos[2].pos = q3;
}

Eigen::Matrix3d DogPhysics::ComputeJacobian(JointState3CRef joint_pos) const
{
    const double c1 = cos(joint_pos[0].pos);
    const double s1 = sin(joint_pos[0].pos);
    const double c2 = cos(joint_pos[1].pos);
    const double s2 = sin(joint_pos[1].pos);
    const double c23 = cos(joint_pos[1].pos + joint_pos[2].pos);
    const double s23 = sin(joint_pos[1].pos + joint_pos[2].pos);

    const double shin_z = thigh_offset_z_ * c2 + shin_offset_z_ * c23;
    const double shin_x = thigh_offset_z_ * s2 + shin_offset_z_ * s23;

    Eigen::Matrix3d jacob;

    jacob(0, 0) = 0.;
    jacob(0, 1) = shin_z;
    jacob(0, 2) = shin_offset_z_ * c23;
    jacob(1, 0) = - shin_z * c1 - hip_len_y_ * s1;
    jacob(1, 1) = shin_x * s1;
    jacob(1, 2) = shin_offset_z_ * s1 * s23;
    jacob(2, 0) = - shin_z * s1 + hip_len_y_ * c1;
    jacob(2, 1) = - shin_x * c1;
    jacob(2, 2) = - shin_offset_z_ * c1 * s23;

    return jacob;
}

Eigen::Vector3d DogPhysics::ComputeFootVel(JointState3CRef joint_stat) const
{
    const Eigen::Matrix3d jacobian = ComputeJacobian(joint_stat);
    const Eigen::Vector3d joint_vel(joint_stat[0].vel,
                                    joint_stat[1].vel,
                                    joint_stat[2].vel);

    return jacobian * joint_vel;
}

void DogPhysics::ComputeJointVel(const Eigen::Vector3d &foot_vel,
                                 JointState3 &joint_stat) const
{
    const Eigen::Matrix3d jacobian = ComputeJacobian(joint_stat);
    Eigen::Vector3d joint_vel;

    if(abs(jacobian.determinant()) < 1e-3)
    {
        joint_vel = (jacobian + Eigen::Matrix3d::Identity() * 1e-3).inverse()
                * foot_vel;
    }
    else
    {
        joint_vel = jacobian.inverse() * foot_vel;
    }

    joint_stat[0].vel = joint_vel(0);
    joint_stat[1].vel = joint_vel(1);
    joint_stat[2].vel = joint_vel(2);
}

void DogPhysics::BuildDynamicsMatrixs(JointState3CRef joint_stat,
                                      Eigen::Matrix3d &H,
                                      Eigen::Matrix3d &C,
                                      Eigen::Matrix3d &G) const
{
    const double c1 = cos(joint_stat[0].pos);
    const double s1 = sin(joint_stat[0].pos);
    const double c2 = cos(joint_stat[1].pos);
    const double s2 = sin(joint_stat[1].pos);
    const double c3 = cos(joint_stat[2].pos);
    const double s3 = sin(joint_stat[2].pos);
    const double c23 = cos(joint_stat[1].pos + joint_stat[2].pos);
    const double s23 = sin(joint_stat[1].pos + joint_stat[2].pos);
    const double vq1 = joint_stat[0].vel;
    const double vq2 = joint_stat[1].vel;
    const double vq3 = joint_stat[2].vel;

    const double thigh_y = hip_len_y_ * c1 - thigh_offset_z_ * s1 * c2;
    const double thigh_z = hip_len_y_ * s1 + thigh_offset_z_ * c1 * c2;

    const double thigh_c_y = com_thigh_y_ + hip_len_y_;
    const double shin_c_x = thigh_offset_z_ * s2 + com_shin_z_ * s23;
    const double shin_c_z = thigh_offset_z_ * c2 + com_shin_z_ * c23;

    const double I2cross = I2yz_ - com_thigh_y_*com_thigh_z_*thigh_mass_;
    const double I2rot = (I2xx_ + I2yy_ - I2zz_) / 2
            + thigh_mass_ * com_thigh_z_ * com_thigh_z_;
    const double I2center = (I2yy_ - I2xx_ + I2zz_) / 2;
    const double I3cross = shin_mass_ * com_shin_z_ * thigh_offset_z_;
    const double I3rot = (I3xx_ + I3yy_ - I3zz_) / 2
            + shin_mass_ * com_shin_z_ * com_shin_z_;
    const double I3center = I3yy_/2 - I3xx_/2 + I3zz_/2;

    H(0, 0) = shin_mass_ * (thigh_z * thigh_z + thigh_y * thigh_y)
            + thigh_mass_ * thigh_c_y * thigh_c_y
            + hip_mass_ * com_hip_y_ * com_hip_y_
            + I1xx_ + (I2xx_ - I2yy_ + I2zz_ + I3xx_ - I3yy_ + I3zz_) / 2
            + 2 * I3cross * c2 * c23
            + I3rot * c23 * c23 + I3center * s23 * s23
            + I2center * s2 * s2 + I2rot * c2 * c2;

    H(0, 1) = I2cross * s2
            - thigh_mass_ * hip_len_y_ * com_thigh_z_ * s2
            - shin_mass_ * hip_len_y_ * shin_c_x;

    H(0, 2) = - shin_mass_ * com_shin_z_ * hip_len_y_ * s23;

    H(1, 1) = I2center + I3center + I2rot + I3rot
            + shin_mass_ * (thigh_offset_z_ * thigh_offset_z_
                            + 2 * com_shin_z_ * thigh_offset_z_ * c3);

    H(1, 2) = I3rot + I3center + I3cross * c3;

    H(2, 2) = I3rot + I3center;

    H(1, 0) = H(0, 1);
    H(2, 0) = H(0, 2);
    H(2, 1) = H(1, 2);

    // before building matrix C, compute its derivative K_ijk first
    // which satisfies: K_ijk = dC(i - 1, j - 1) / dvq_k
    // and K_ijk = K_ikj

    //K111 = 0;
    const double K112 = - I3cross * (c2 * s23 + s2 * c23)
            + (I3center - I3rot) * s23 * c23
            + (I2center - I2rot
               - shin_mass_ * thigh_offset_z_ * thigh_offset_z_) * c2 * s2;
    const double K113 = - I3cross * c2 * s23
            + (I3center - I3rot) * s23 * c23;
    const double K122 = - shin_mass_ * hip_len_y_ * shin_c_z
            + (I2cross - thigh_mass_ * com_thigh_z_ * hip_len_y_) * c2;
    const double K123 = - shin_mass_ * com_shin_z_ * hip_len_y_ * c23;
    // K133 = K123;
    // K211 = - K112;
    // K212 = 0;
    // K213 = 0;
    // K222 = 0;
    const double K223 = - I3cross * s3;
    // K233 = K223;
    // K311 = - K113;
    // K312 = 0;
    // K313 = 0;
    // K322 = - K223;
    // K323 = 0;
    // K333 = 0;

    C(0, 0) =                 K112 * vq2 +  K113 * vq3;
    C(0, 1) =   K112 * vq1 +  K122 * vq2 +  K123 * vq3;
    C(0, 2) =   K113 * vq1 +  K123 * (vq2 +        vq3);
    C(1, 0) = - K112 * vq1                            ;
    C(1, 1) =                            +  K223 * vq3;
    C(1, 2) =              +  K223 * (vq2 +        vq3);
    C(2, 0) = - K113 * vq1                            ;
    C(2, 1) =              -  K223 * vq2              ;
    C(2, 2) =                                        0;

    G(0, 0) = 0;

    G(0, 1) =
              hip_mass_ * com_hip_y_ * s1
            + thigh_mass_ * (com_thigh_z_ * c1 * c2 + thigh_c_y * s1)
            + shin_mass_ * (com_shin_z_ * c1 * c23 + thigh_z);

    G(0, 2) =
            - hip_mass_ * com_hip_y_ * c1
            + thigh_mass_ * (com_thigh_z_ * s1 * c2 - thigh_c_y * c1)
            + shin_mass_ * (com_shin_z_ * s1 * c23 - thigh_y);

    G(1, 0) =
            - thigh_mass_ * com_thigh_z_ * c2
            - shin_mass_ * shin_c_z;

    G(1, 1) =
            - thigh_mass_ * com_thigh_z_ * s1 * s2
            - shin_mass_ * shin_c_x * s1;

    G(1, 2) =
              thigh_mass_ * com_thigh_z_ * c1 * s2
            + shin_mass_ * shin_c_x * c1;

    G(2, 0) =
            - shin_mass_ * com_shin_z_ * c23;

    G(2, 1) =
            - shin_mass_ * com_shin_z_ * s1 * s23;

    G(2, 2) =
              shin_mass_ * com_shin_z_ * c1 * s23;
}

Eigen::Vector3d DogPhysics::ComputeTorque(JointState3CRef joint_stat,
                                          const Eigen::Vector3d &gravity) const
{
    const Eigen::Vector3d vel(joint_stat[0].vel,
                              joint_stat[1].vel,
                              joint_stat[2].vel);
    const Eigen::Vector3d acc(joint_stat[0].acc,
                              joint_stat[1].acc,
                              joint_stat[2].acc);
    Eigen::Matrix3d H;
    Eigen::Matrix3d C;
    Eigen::Matrix3d G;
    BuildDynamicsMatrixs(joint_stat, H, C, G);

    return H * acc + C * vel + G * gravity;
}

Eigen::Vector3d DogPhysics::ComputeFriction(JointState3CRef joint_stat) const
{
    Eigen::Vector3d f;

    for(int i = 0; i < 3; i++)
    {
        const double vel = joint_stat[i].vel;

        if(vel > vel_deadband_)
            f(i) = - friction_ - vel * damping_;
        else if(vel < - vel_deadband_)
            f(i) = friction_ - vel * vel_deadband_;
        else
            f(i) = - vel * damping_;
    }

    return f;
}

} /* physics */

} /* dog_control */
