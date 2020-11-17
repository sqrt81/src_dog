#include "dog_control/control/ConfSpaceTraj.h"

#include "dog_control/physics/DogModel.h"
#include "dog_control/utils/CubicSplineImpl.hpp"
#include "dog_control/utils/Math.h"
#include "dog_control/utils/MiniLog.h"

namespace dog_control
{

namespace control
{

ConfSpaceTraj::ConfSpaceTraj(double beg_t,
                             double end_t,
                             const Eigen::Vector3d &beg_pos,
                             const Eigen::Vector3d &beg_vel,
                             message::LegConfigCRef beg_conf,
                             const Eigen::Vector3d &end_pos,
                             const Eigen::Vector3d &end_vel,
                             message::LegConfigCRef end_conf,
                             const physics::DogModel &model)
 : model_(model), leg_(beg_conf.foot_name)
{
    CHECK(end_t > beg_t) << "[SwingTraj]: traj ends before it starts!";
    CHECK(leg_ == end_conf.foot_name) << "[SwingTraj]: inconsistent leg name!";

    begin_time_ = beg_t;
    end_time_ = end_t;

    const Eigen::Vector3d beg_jpos
            = model_.InverseKinematics(leg_, beg_pos,
                                       beg_conf.knee_outwards,
                                       beg_conf.hip_outwards);
    const Eigen::Vector3d end_jpos
            = model_.InverseKinematics(leg_, end_pos,
                                       end_conf.knee_outwards,
                                       end_conf.hip_outwards);

    Eigen::Matrix3d inv_j;
    bool invertable;
    model_.ComputeJacobian(beg_conf.foot_name, beg_jpos)
            .computeInverseWithCheck(inv_j, invertable, utils::precision);
    const Eigen::Vector3d beg_jvel
            = invertable ? Eigen::Vector3d(inv_j * beg_vel)
                         : Eigen::Vector3d::Zero();

    model_.ComputeJacobian(end_conf.foot_name, end_jpos)
            .computeInverseWithCheck(inv_j, invertable, utils::precision);
    Eigen::Vector3d end_jvel
            = invertable ? Eigen::Vector3d(inv_j * end_vel)
                         : Eigen::Vector3d::Zero();

    joint_traj_ = utils::CubicSpline<Eigen::Vector3d>(
                beg_t, beg_jpos, beg_jvel,
                end_t, end_jpos, end_jvel);
}

void ConfSpaceTraj::Sample(double t,
                           Eigen::Vector3d &local_pos,
                           Eigen::Vector3d &local_vel,
                           Eigen::Vector3d &local_acc,
                           bool &hip_outwards,
                           bool &knee_outwards) const
{
    Eigen::Vector3d jpos;
    Eigen::Vector3d jvel;

    joint_traj_.Sample(t, jpos, jvel);

    local_acc.setZero(); // don't know how to compute it
    local_pos = model_.ComputeLocalPos(leg_, jpos);
    local_vel = model_.ComputeJacobian(leg_, jpos) * jvel;

    model_.GetLegConfig(jpos, knee_outwards, hip_outwards);

//    if (leg_ == message::FR)
//        LOG(DEBUG) << "desired: " << jpos.transpose();
}

} /* control */

} /* dog_control */
