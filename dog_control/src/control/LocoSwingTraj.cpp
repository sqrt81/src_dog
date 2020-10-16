#include "dog_control/control/LocoSwingTraj.h"

#include "dog_control/utils/CubicSplineImpl.hpp"
#include "dog_control/utils/MiniLog.h"

namespace dog_control
{

namespace control
{

LocoSwingTraj::LocoSwingTraj(double beg_t,
                             double end_t,
                             const Eigen::Vector3d &beg_pos,
                             const Eigen::Vector3d &beg_vel,
                             const Eigen::Vector3d &end_pos,
                             const Eigen::Vector3d &end_vel,
                             const Eigen::Vector3d &raise_offset)
{
    CHECK(end_t > beg_t) << "[SwingTraj]: traj ends before it starts!";

    begin_time_ = beg_t;
    end_time_ = end_t;

    const double dt = end_t - beg_t;
    half_time_ = (beg_t + end_t) * 0.5;
    const Eigen::Vector3d half_pos = (beg_pos + end_pos) * 0.5
            + (beg_vel - end_vel) * (0.125 * dt) + raise_offset;
    const Eigen::Vector3d half_vel = (end_pos - beg_pos) * (1.5 / dt)
            - (beg_vel + end_vel) * 0.25;

    rise_ = utils::CubicSpline<Eigen::Vector3d>(
                beg_t, beg_pos, beg_vel,
                half_time_, half_pos, half_vel);
    fall_ = utils::CubicSpline<Eigen::Vector3d>(
                half_time_, half_pos, half_vel,
                end_t, end_pos, end_vel);
}

void LocoSwingTraj::Sample(double t,
                           Eigen::Vector3d &local_pos,
                           Eigen::Vector3d &local_vel,
                           Eigen::Vector3d &local_acc) const
{
    if (t < half_time_)
    {
        if (t < begin_time_)
            t = begin_time_;

        rise_.Sample(t, local_pos, local_vel, local_acc);
    }
    else
    {
        if (t > end_time_)
            t = end_time_;

        fall_.Sample(t, local_pos, local_vel, local_acc);
    }
}

} /* control */

} /* dog_control */
