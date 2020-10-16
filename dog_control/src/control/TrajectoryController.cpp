#include "dog_control/control/TrajectoryController.h"

#include "dog_control/hardware/ClockBase.h"
#include "dog_control/utils/CubicSpline.h"
#include "dog_control/utils/CubicSplineImpl.hpp"
#include "dog_control/utils/MiniLog.h"
#include "dog_control/utils/QueueImpl.hpp"

namespace dog_control
{

namespace control
{

void TrajectoryController::Initialize(utils::ParamDictCRef dict)
{
    dt_ = ReadParOrDie(dict, PARAM_WITH_NS(control_period, control));
    traj_record_len_ = ReadParOrDie(dict, PARAM_WITH_NS(
            traj_len, control/TRAJECTORY)) / dt_;
    traj_beg_time_ = - 1.;
}

void TrajectoryController::ConnectClock(
        boost::shared_ptr<hardware::ClockBase> clock)
{
    clock_ptr_ = clock;
}

void TrajectoryController::SetTorsoTrajectory(const TorsoTraj &torso_traj)
{
    unsigned int iter = 0;

    for (; iter < torso_traj.size(); iter++)
    {
        // ignore the outdated trajectory points
        if (torso_traj[iter].stamp > traj_beg_time_)
            break;
    }

    if (torso_traj.empty())
    {
        LOG(WARN) << "[TrajController] torso_traj has zero size.";
        return;
    }

    if (iter == torso_traj.size())
    {
        LOG(WARN) << "[TrajController] torso_traj is outdated.";
        return;
    }

    // Find the index of the first element in torso_traj_
    // to be replaced.
    const int index
            = std::max(static_cast<int>((torso_traj[iter].stamp
                                         - traj_beg_time_) / dt_) + 1,
                       torso_traj_.size());

    torso_traj_.EraseTail(torso_traj_.size() - index);

    double last_time = traj_beg_time_ + (index - 1) * dt_;
    message::FloatingBaseState last_state;

    if (torso_traj_.size())
        last_state = torso_traj_[index - 1].state;
    else // first run
    {
        last_state = torso_traj[iter].state;
        last_time = torso_traj[iter].stamp;
        traj_beg_time_ = last_time;
        iter++;
    }

    last_state.linear_vel = last_state.rot * last_state.linear_vel;
    last_state.rot_vel = last_state.rot * last_state.rot_vel;
    double sample_time = last_time + dt_;

    for (; iter < torso_traj.size(); iter++)
    {
        message::FloatingBaseState next_state = torso_traj[iter].state;
        next_state.linear_vel = next_state.rot * next_state.linear_vel;
        next_state.rot_vel = next_state.rot * next_state.rot_vel;
        const double next_time = torso_traj[iter].stamp;

        utils::CubicSpline<Eigen::Vector3d> trans(
                    last_time, last_state.trans, last_state.linear_vel,
                    next_time, next_state.trans, next_state.linear_vel);

        FBState cur_state;
        cur_state.rot_acc
                = (next_state.rot_vel - last_state.rot_vel)
                / (next_time - last_time);

        while (sample_time < next_time)
        {
            // keep the trajectory length short.
            if (torso_traj_.size() >= traj_record_len_)
                return;

            const double ratio
                    = (sample_time - last_time) / (next_time - last_time);

            trans.Sample(sample_time,
                         cur_state.state.trans,
                         cur_state.state.linear_vel,
                         cur_state.linear_acc);

            cur_state.state.rot = last_state.rot.slerp(ratio, next_state.rot);
            cur_state.state.rot_vel = last_state.rot_vel
                    + ratio * (next_state.rot_vel - last_state.rot_vel);

            FBState rotated_state;
            rotated_state.state.trans = cur_state.state.trans;
            rotated_state.state.rot = cur_state.state.rot;
            rotated_state.state.linear_vel
                    = cur_state.state.rot.conjugate()
                    * cur_state.state.linear_vel;
            rotated_state.state.rot_vel
                    = cur_state.state.rot.conjugate()
                    * cur_state.state.rot_vel;
            rotated_state.linear_acc
                    = cur_state.state.rot.conjugate()
                    * cur_state.linear_acc;
            rotated_state.rot_acc
                    = cur_state.state.rot.conjugate()
                    * cur_state.rot_acc;
            torso_traj_.Push(rotated_state);

            sample_time += dt_;
        }

        last_time = next_time;
        last_state = next_state;
    }
}

TrajectoryController::FBState
TrajectoryController::GetTorsoState(double t) const
{
    if (t < traj_beg_time_)
        return torso_traj_.Head();

    if (t > traj_beg_time_ + dt_ * torso_traj_.size())
        return  torso_traj_.Tail();

    const int index = static_cast<int>((t - traj_beg_time_) / dt_);
    return torso_traj_[index];
}

void TrajectoryController::GetTorsoTraj(
        const std::vector<double> &time_stamp,
        std::vector<FBState> &traj) const
{
    traj.clear();
    traj.reserve(time_stamp.size());

    for (const double t : time_stamp)
    {
        if (t < traj_beg_time_)
            traj.push_back(torso_traj_.Head());
        else if (t > traj_beg_time_ + dt_ * torso_traj_.size())
            traj.push_back(torso_traj_.Tail());
        else
        {
            const int index = static_cast<int>((t - traj_beg_time_) / dt_);
            traj.push_back(torso_traj_[index]);
        }
    }
}

void TrajectoryController::SampleTrajFromNow(int sample_cnt, double interval,
                                             std::vector<MPCState> &traj) const
{
    boost::shared_ptr<hardware::ClockBase> clock = clock_ptr_.lock();
    CHECK(clock) << "[TrajController] clock is not set!";

    const double cur_time = clock->Time();

    traj.clear();
    traj.reserve(sample_cnt);

    for (int i = 0; i < sample_cnt; i++)
    {
        const double t = cur_time + i * interval;

        if (t < traj_beg_time_)
            traj.push_back(torso_traj_.Head().state);
        else if (t > traj_beg_time_ + dt_ * torso_traj_.size())
            traj.push_back(torso_traj_.Tail().state);
        else
        {
            const int index = static_cast<int>((t - traj_beg_time_) / dt_);
            traj.push_back(torso_traj_[index].state);
        }
    }
}

void TrajectoryController::Update()
{
    // don't update if not initialized.
    if (torso_traj_.size() == 0)
        return;

    boost::shared_ptr<hardware::ClockBase> clock = clock_ptr_.lock();
    CHECK(clock) << "[TrajController] clock is not set!";

    const double cur_time = clock->Time();

    // remove outdated trajectory points
    int index = static_cast<int>(
                (cur_time - traj_beg_time_) / dt_);

    traj_beg_time_ += index * dt_;

    // leave at least one element in trajectory
    if (index >= torso_traj_.size())
        index = torso_traj_.size() - 1;

    torso_traj_.EraseHead(index);
}

} /* control */

} /* dog_control */
