#include "dog_control/control/TrajectoryController.h"

#include "dog_control/hardware/ClockBase.h"
#include "dog_control/physics/DogModel.h"
#include "dog_control/utils/CubicSpline.h"
#include "dog_control/utils/CubicSplineImpl.hpp"
#include "dog_control/utils/QuatSplineImpl.hpp"
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

void TrajectoryController::ConnectModel(
        boost::shared_ptr<physics::DogModel> model)
{
    model_ptr_ = model;
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

    const bool first_run = (torso_traj_.size() == 0);
    double sample_time = last_time + dt_;

    if (first_run)
    {
        last_state = torso_traj[iter].state;
        last_time = torso_traj[iter].stamp;
        traj_beg_time_ = last_time;
        sample_time = last_time; // sample the begin point of the traj
        iter++;
    }
    else
        last_state = torso_traj_[index - 1].state;

    // The CubicSpline for Quaternion is tailored for local
    // frame rotation representation. So the rotational velocity
    // needs not to be converted into global frame.
//    last_state.rot_vel = last_state.rot * last_state.rot_vel;
    last_state.linear_vel = last_state.rot * last_state.linear_vel;

    for (; iter < torso_traj.size(); iter++)
    {
        message::FloatingBaseState next_state = torso_traj[iter].state;
//        next_state.rot_vel = next_state.rot * next_state.rot_vel;
        next_state.linear_vel = next_state.rot * next_state.linear_vel;
        const double next_time = torso_traj[iter].stamp;

        utils::CubicSpline<Eigen::Vector3d> trans(
                    last_time, last_state.trans, last_state.linear_vel,
                    next_time, next_state.trans, next_state.linear_vel);

        utils::CubicSpline<Eigen::Quaterniond, Eigen::Vector3d> rot(
                    last_time, last_state.rot, last_state.rot_vel,
                    next_time, next_state.rot, next_state.rot_vel);

        FBState cur_state;

        while (sample_time < next_time)
        {
            // keep the trajectory length short.
            if (torso_traj_.size() >= traj_record_len_)
                break;

            trans.Sample(sample_time,
                         cur_state.state.trans,
                         cur_state.state.linear_vel,
                         cur_state.linear_acc);
            rot.Sample(sample_time,
                       cur_state.state.rot,
                       cur_state.state.rot_vel,
                       cur_state.rot_acc);

            // rotate back into local frame
            cur_state.state.linear_vel
                    = cur_state.state.rot.conjugate()
                    * cur_state.state.linear_vel;
            cur_state.linear_acc
                    = cur_state.state.rot.conjugate()
                    * cur_state.linear_acc;
            torso_traj_.Push(cur_state);
            sample_time += dt_;
        }

        if (torso_traj_.size() >= traj_record_len_)
            break;

        last_time = next_time;
        last_state = next_state;
    }

    if (first_run)
    {
        // init foot pos for first run
        boost::shared_ptr<physics::DogModel> model = model_ptr_.lock();
        CHECK(model) << "[TrajController] Model is not set!";

        for (int i = 0; i < 4; i++)
            if (!swing_traj_[i])
            {
                foot_pos_[i]
                        = model->FootPos(static_cast<message::LegName>(i));
            }
    }
}

void TrajectoryController::SetFootTrajectory(
        message::LegName foot_name, boost::shared_ptr<FootSwingTrajBase> traj)
{
    CHECK(VALID_LEGNAME(foot_name));

    if (traj->EndTime() < traj_beg_time_)
        LOG(WARN) << "[TrajController] foot traj is outdated, rejected.";
    else
        swing_traj_[foot_name] = traj;
}

TrajectoryController::FBState
TrajectoryController::GetTorsoState(double t) const
{
    return torso_traj_[GetIndexForTime(t)];
}

void TrajectoryController::GetCurLocalFootState(
        message::LegName foot_name,
        Eigen::Vector3d &pos,
        Eigen::Vector3d &vel,
        bool &hip_out,
        bool &knee_out) const
{
    CHECK(VALID_LEGNAME(foot_name));

    const message::FloatingBaseState& cur_state = torso_traj_.Head().state;

    if (!swing_traj_[foot_name]
            || traj_beg_time_ < swing_traj_[foot_name]->BeginTime())
    {
        pos = cur_state.rot.conjugate()
                * (foot_pos_[foot_name] - cur_state.trans);
        vel = - cur_state.linear_vel;

        // leg configuration is only modified during swinging
        (void) hip_out;
        (void) knee_out;
    }
    else
    {
        Eigen::Vector3d acc;
        swing_traj_[foot_name]->Sample(traj_beg_time_, pos, vel, acc,
                                       hip_out, knee_out);
    }
}

void TrajectoryController::GetFootState(double t,
                                        message::LegName foot_name,
                                        Eigen::Vector3d &pos,
                                        Eigen::Vector3d &vel,
                                        Eigen::Vector3d &acc,
                                        bool &contact) const
{
    CHECK(VALID_LEGNAME(foot_name));

    if (swing_traj_[foot_name])
    {
        const FootSwingTrajBase& traj = *swing_traj_[foot_name];

        Eigen::Vector3d local_pos;
        Eigen::Vector3d local_vel;
        Eigen::Vector3d local_acc;
        bool hip_out;
        bool knee_out;
        traj.Sample(t, local_pos, local_vel, local_acc, hip_out, knee_out);

        if (t > traj.EndTime())
        {
            // In this case, the foot has done swinging. So the foot global
            // position equals to its position when swing just finished.
            // The foot's velocity and acceleration are zero (since it stays
            // in contact with ground).
            const FBState& stat = torso_traj_[GetIndexForTime(traj.EndTime())];
            pos = stat.state.trans + stat.state.rot * local_pos;
            vel.setZero();
            acc.setZero();
            contact = true;

            return;
        }

        else if (t >= traj.BeginTime())
        {
            // The foot is swinging. Convert into global frame.
            const FBState& stat = torso_traj_[GetIndexForTime(t)];
            pos = stat.state.trans + stat.state.rot * local_pos;
            vel = stat.state.rot * (stat.state.linear_vel + local_vel
                                    + stat.state.rot_vel.cross(local_pos));
            acc = stat.state.rot * (
                        stat.linear_acc + local_acc
                        - stat.state.rot_vel.squaredNorm() * local_pos
                        + stat.rot_acc.cross(local_pos)
                        + 2 * stat.state.rot_vel.cross(local_vel));
            contact = false;

            return;
        }
    }

    // If the following lines are executed, it suggests either
    // there is no swing trajectory for this foot or the swing
    // trajectory has not yet begun at the sample time.
    // So the foot has not moved in global frame.
    pos = foot_pos_[foot_name];
    vel.setZero();
    acc.setZero();
    contact = true;

    return;
}

void TrajectoryController::GetTorsoTraj(
        const std::vector<double> &time_stamp,
        std::vector<FBState> &traj) const
{
    traj.clear();
    traj.reserve(time_stamp.size());

    for (const double t : time_stamp)
    {
        traj.push_back(torso_traj_[GetIndexForTime(t)]);
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

        traj.push_back(torso_traj_[GetIndexForTime(t)].state);
    }
}

void TrajectoryController::SampleFootStateFromNow(
        int sample_cnt, double interval,
        std::vector<std::array<Eigen::Vector3d, 4>> &pos_seq,
        std::vector<std::array<bool, 4>> &contact_seq) const
{
    boost::shared_ptr<hardware::ClockBase> clock = clock_ptr_.lock();
    CHECK(clock) << "[TrajController] clock is not set!";

    const double cur_time = clock->Time();

    pos_seq.resize(sample_cnt);
    contact_seq.resize(sample_cnt);

    const FootSwingTrajBase* traj[4];

    for (int j = 0; j < 4; j++)
        if (swing_traj_[j])
            traj[j] = swing_traj_[j].get();
        else
            traj[j] = nullptr;

    for (int i = 0; i < sample_cnt; i++)
    {
        const double t = cur_time + i * interval;
        const FBState& cur_stat = torso_traj_[GetIndexForTime(t)];

        for (int j = 0; j < 4; j++)
        {
            if (traj[j] == nullptr || t < traj[j]->BeginTime())
            {
                // no swing traj given, or swinging hasn't begun
                pos_seq[i][j] = foot_pos_[j];
                contact_seq[i][j] = true;
            }
            else
            {
                Eigen::Vector3d local_pos;
                Eigen::Vector3d local_vel;
                Eigen::Vector3d local_acc;
                bool hip_out;
                bool knee_out;
                traj[j]->Sample(t, local_pos, local_vel, local_acc,
                                hip_out, knee_out);

                if (t > traj[j]->EndTime())
                {
                    // swing finished
                    const FBState& end_stat
                            = torso_traj_[GetIndexForTime(traj[j]->EndTime())];
                    pos_seq[i][j] = end_stat.state.trans
                                  + end_stat.state.rot * local_pos;
                    contact_seq[i][j] = true;
                }
                else
                {
                    // swinging
                    pos_seq[i][j] = cur_stat.state.trans
                                  + cur_stat.state.rot * local_pos;
                    contact_seq[i][j] = false;
                }
            }
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

    const FBState& beg_stat = torso_traj_.Head();

    // remove a trajectory if it has been executed
    for (int i = 0; i < 4; i++)
    {
        if (swing_traj_[i] && swing_traj_[i]->EndTime() < traj_beg_time_)
        {
            Eigen::Vector3d pos;
            Eigen::Vector3d vel;
            Eigen::Vector3d acc;
            bool hip_out;
            bool knee_out;
            swing_traj_[i]->Sample(traj_beg_time_, pos, vel, acc,
                                   hip_out, knee_out);
            foot_pos_[i] = beg_stat.state.trans + beg_stat.state.rot * pos;

            swing_traj_[i].reset();
        }
    }
}

inline int TrajectoryController::GetIndexForTime(double t) const
{
    int index = static_cast<int>((t - traj_beg_time_) / dt_);

    if (t < traj_beg_time_)
        index = 0;
    else if (t >= traj_beg_time_ + dt_ * torso_traj_.size())
        index = torso_traj_.size() - 1;

    return index;
}

} /* control */

} /* dog_control */
