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
    raise_offset_ = raise_offset;

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
                           Eigen::Vector3d &local_acc,
                           message::LegConfigRef conf) const
{
    (void) conf;

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

bool LocoSwingTraj::DecideEnd(double t, bool in_contact)
{
    if (in_contact && t > (begin_time_ + end_time_) / 2)
        return true;

    return (t > end_time_);
}

void LocoSwingTraj::Replan(double replan_t,
                           const Eigen::Vector3d &end_pos,
                           const Eigen::Vector3d &end_vel)
{
    const double t1 = end_time_ - replan_t;
    const double t2 = half_time_ - replan_t;
    const double t2_2 = t2 * t2;
    const double inv_t1_2 = 1. / (t1 * t1);
    const double inv_t1_3 = inv_t1_2 / t1;

    Eigen::Vector3d end_pos_diff;
    Eigen::Vector3d end_vel_diff;
    Eigen::Vector3d half_pos;
    Eigen::Vector3d half_vel;

    fall_.Sample(end_time_, end_pos_diff, end_vel_diff);
    fall_.Sample(half_time_, half_pos, half_vel);

    // calculate end difference and half time target
    end_pos_diff = end_pos - end_pos_diff;
    end_vel_diff = end_vel - end_vel_diff;
    half_pos += t2_2 * (3 * t1 - 2 * t2) * inv_t1_3 * end_pos_diff
              + t2_2 * (t2 - t1) * inv_t1_2 * end_vel_diff;
    half_vel += 6 * t2 * (t1 - t2) * inv_t1_3 * end_pos_diff
              + t2 * (3 * t2 - 2 * t1) * inv_t1_2 * end_vel_diff;

    if (replan_t < half_time_)
    {
        Eigen::Vector3d replan_pos;
        Eigen::Vector3d replan_vel;

        rise_.Sample(replan_t, replan_pos, replan_vel);
        rise_ = utils::CubicSpline<Eigen::Vector3d>(
                    replan_t, replan_pos, replan_vel,
                    half_time_, half_pos, half_vel);
    }

    fall_ = utils::CubicSpline<Eigen::Vector3d>(
                half_time_, half_pos, half_vel,
                end_time_, end_pos, end_vel);
}

} /* control */

} /* dog_control */
