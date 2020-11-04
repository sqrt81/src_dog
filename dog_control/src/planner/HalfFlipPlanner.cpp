#include "dog_control/planner/HalfFlipPlanner.h"

#include "dog_control/control/ConfSpaceTraj.h"
#include "dog_control/control/TrajectoryController.h"
#include "dog_control/control/FootPosController.h"
#include "dog_control/hardware/ClockBase.h"
#include "dog_control/physics/DogModel.h"
#include "dog_control/utils/MiniLog.h"

namespace dog_control
{

namespace planner
{

void HalfFlipPlanner::Initialize(utils::ParamDictCRef dict)
{
    forward_vel_ = ReadParOrDie(dict, PARAM_WITH_NS(forward_vel,
                                                    planner/HALF_FLIP));
    aerial_time_ = ReadParOrDie(dict, PARAM_WITH_NS(aerial_time,
                                                    planner/HALF_FLIP));
    stance_time_ = ReadParOrDie(dict, PARAM_WITH_NS(stance_time,
                                                    planner/HALF_FLIP));
    rot_factor_ = ReadParOrDie(dict, PARAM_WITH_NS(rot_factor,
                                                   planner/HALF_FLIP));
    step_width_ = 0.283 * 2;
    flip_time_ = aerial_time_ + stance_time_ * 2;
    torso_height_ = 0.15;
    gravity_ = 9.8;

    upside_down_ = false;
    last_update_time_ = -10.;
}

void HalfFlipPlanner::ConnectTraj(
        boost::shared_ptr<control::TrajectoryController> traj)
{
    traj_ptr_ = traj;
}

void HalfFlipPlanner::ConnectFoot(
        boost::shared_ptr<control::FootPosController> foot_pos)
{
    foot_pos_ptr_ = foot_pos;
}

void HalfFlipPlanner::ConnectClock(
        boost::shared_ptr<hardware::ClockBase> clock)
{
    clock_ptr_ = clock;
}

void HalfFlipPlanner::ConnectModel(
        boost::shared_ptr<physics::DogModel> model)
{
    model_ptr_ = model;
}

void HalfFlipPlanner::Update()
{
    boost::shared_ptr<hardware::ClockBase> clock = clock_ptr_.lock();
    CHECK(clock) << "[HalfFlipPlanner] clock is not set!";
    const double cur_time = clock->Time();

    boost::shared_ptr<physics::DogModel> model = model_ptr_.lock();
    CHECK(model) << "[HalfFlipPlanner] model is not set!";

    boost::shared_ptr<control::FootPosController> foot_ctrl
            = foot_pos_ptr_.lock();
    CHECK(foot_ctrl) << "[HalfFlipPlanner] foot_ctrl is not set!";

    boost::shared_ptr<control::TrajectoryController> traj
            = traj_ptr_.lock();
    CHECK(traj) << "[HalfFlipPlanner] traj is not set!";

    message::LegConfiguration cur_conf;

    if (cur_time > last_update_time_ + flip_time_)
    {
        const double raise_vel = 0.5 * aerial_time_ * gravity_;
        const double rot_vel = rot_factor_ * M_PI
                / (aerial_time_ + stance_time_);

        std::vector<message::StampedFloatingBaseState> torso_traj;
        message::StampedFloatingBaseState state;
        state.stamp = cur_time + 0.01;
        state.state.trans = model->TorsoState().trans;
        state.state.trans.z() = torso_height_;
        state.state.linear_vel = Eigen::Vector3d(0, 0, 0);
        state.state.rot_vel = Eigen::Vector3d(0, 0, 0);

        if (upside_down_)
            state.state.rot = Eigen::AngleAxisd(- M_PI,
                                                Eigen::Vector3d::UnitX());
        else
            state.state.rot = Eigen::Quaterniond::Identity();

        torso_traj.push_back(state);

        state.stamp = cur_time + stance_time_;
        state.state.trans.y() += forward_vel_ * stance_time_ / 2;
        state.state.trans.z()
                = stance_time_ * raise_vel / 2
                + torso_height_;
        state.state.linear_vel = Eigen::Vector3d(0, forward_vel_, raise_vel);
        state.state.rot_vel = Eigen::Vector3d(- rot_vel, 0, 0);

        if (upside_down_)
            state.state.rot
                    = Eigen::AngleAxisd(M_PI - rot_vel * stance_time_ / 2,
                                        Eigen::Vector3d::UnitX());
        else
            state.state.rot
                    = Eigen::AngleAxisd(- rot_vel * stance_time_ / 2,
                                        Eigen::Vector3d::UnitX());

        state.state.linear_vel
                = state.state.rot.conjugate() * state.state.linear_vel;
        torso_traj.push_back(state);

        state.stamp = cur_time + flip_time_ - stance_time_;
        state.state.trans.y() += forward_vel_ * aerial_time_;
        state.state.trans.z()
                = stance_time_ * raise_vel / 2
                + torso_height_;
        state.state.linear_vel
                = Eigen::Vector3d(0, forward_vel_, - raise_vel);
        state.state.rot_vel = Eigen::Vector3d(- rot_vel, 0, 0);

        if (upside_down_)
            state.state.rot
                    = Eigen::AngleAxisd(rot_vel * stance_time_ / 2,
                                        Eigen::Vector3d::UnitX());
        else
            state.state.rot
                    = Eigen::AngleAxisd(- M_PI + rot_vel * stance_time_ / 2,
                                        Eigen::Vector3d::UnitX());

        state.state.linear_vel
                = state.state.rot.conjugate() * state.state.linear_vel;
        torso_traj.push_back(state);

        state.stamp = cur_time + flip_time_;
        state.state.trans.y() += forward_vel_ * stance_time_ / 2;
        state.state.trans.z() = torso_height_;
        state.state.linear_vel = Eigen::Vector3d(0, 0, 0);
        state.state.rot_vel = Eigen::Vector3d(0, 0, 0);

        if (upside_down_)
            state.state.rot = Eigen::Quaterniond::Identity();
        else
            state.state.rot
                    = Eigen::AngleAxisd(- M_PI,
                                        Eigen::Vector3d::UnitX());

        state.state.linear_vel
                = state.state.rot.conjugate() * state.state.linear_vel;
        torso_traj.push_back(state);

        traj->SetTorsoTrajectory(torso_traj);

        message::FloatingBaseStateWithAcc s
                = traj->GetTorsoState(cur_time + stance_time_);

        double end_height = torso_height_ * (upside_down_ ? - 1 : 1);

        for (int i = 0; i < 4; i++)
        {
            const message::LegName name = static_cast<message::LegName>(i);
            Eigen::Vector3d beg_pos;
            Eigen::Vector3d vel;
            Eigen::Vector3d acc;
            bool contact;

            traj->GetFootState(cur_time + stance_time_, name,
                               beg_pos, vel, acc, contact);
            beg_pos = s.state.rot.conjugate() * (beg_pos - s.state.trans);
            message::LegConfiguration conf_beg;
            conf_beg.foot_name = name;
            conf_beg.hip_outwards = !upside_down_;
            conf_beg.knee_outwards = !upside_down_;
            message::LegConfiguration conf_end;
            conf_end.foot_name = name;
            conf_end.hip_outwards = upside_down_;
            conf_end.knee_outwards = upside_down_;
            Eigen::Vector3d end_pos(0.283 * (i < 2 ? 1 : - 1),
                                    0.118 * (i % 2 == 0 ? 1 : - 1),
                                    end_height);
            Eigen::Vector3d beg_vel(0, 0, - beg_pos.z());
            Eigen::Vector3d end_vel(0, 0, - end_pos.z());
            traj->SetFootTrajectory(
                        name,
                        boost::shared_ptr<control::ConfSpaceTraj>(
                            new control::ConfSpaceTraj(
                                cur_time + stance_time_,
                                cur_time + stance_time_ + aerial_time_ * 0.8,
                                beg_pos, beg_vel, conf_beg,
                                end_pos, end_vel, conf_end, *model)));

            contact_[i] = true;

            cur_conf.foot_name = name;
            cur_conf.kd = 0.1;
            cur_conf.kp = 0.3;
            cur_conf.hip_outwards = !upside_down_;
            cur_conf.knee_outwards = !upside_down_;
            foot_ctrl->ChangeFootControlMethod(cur_conf);
        }

        upside_down_ = !upside_down_;
        last_update_time_ = cur_time;
        airborn_ = false;
    }
    else if ((cur_time - last_update_time_ > stance_time_) && !airborn_)
    {
        // set pd control

        for (int i = 0; i < 4; i++)
        {
            cur_conf.foot_name = static_cast<message::LegName>(i);
            cur_conf.kd = 4;
            cur_conf.kp = 30;
            cur_conf.hip_outwards = upside_down_;
            cur_conf.knee_outwards = upside_down_;
            foot_ctrl->ChangeFootControlMethod(cur_conf);
            contact_[i] = false;
        }

        airborn_ = true;
    }
    else if (cur_time - last_update_time_ > flip_time_ - stance_time_)
    {
        // contact detect
        const auto& contact = model->FootContact();

        for (int i = 0; i < 4; i++)
        {
            if (!contact_[i] && contact[i])
            {
                traj->UpdateFootPos();
                contact_[i] = contact[i];
                traj->SetFootTrajectory(
                            static_cast<message::LegName>(i),
                            boost::shared_ptr<control::FootSwingTrajBase>());
                cur_conf.foot_name = static_cast<message::LegName>(i);
                cur_conf.kd = 0.1;
                cur_conf.kp = 0.3;
                cur_conf.hip_outwards = !upside_down_;
                cur_conf.knee_outwards = !upside_down_;
                foot_ctrl->ChangeFootControlMethod(cur_conf);

                if (contact_[0] && contact_[1] && contact_[2] && contact_[3])
                {
                    std::vector<message::StampedFloatingBaseState> torso_traj;
                    message::StampedFloatingBaseState state;
                    state.stamp = cur_time + 0.01;
                    state.state.trans
                            = model->FootPos(message::FL)
                            + model->FootPos(message::FR)
                            + model->FootPos(message::BL)
                            + model->FootPos(message::BR);
                    state.state.trans /= 4;
                    state.state.trans.z() = torso_height_;

                    const Eigen::Vector3d foot_ori
                            = model->FootPos(message::FL)
                            + model->FootPos(message::FR)
                            - model->FootPos(message::BL)
                            - model->FootPos(message::BR);
                    const double yaw = atan2(foot_ori.y(), foot_ori.x());
                    state.state.rot
                            = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                            * Eigen::AngleAxisd(upside_down_ ? - M_PI : 0,
                                                Eigen::Vector3d::UnitX());
                    state.state.linear_vel = Eigen::Vector3d(0, 0, 0);
                    state.state.rot_vel = Eigen::Vector3d(0, 0, 0);
                    torso_traj.push_back(state);
                    state.stamp += 0.1;
                    torso_traj.push_back(state);
                    traj->ClearTorsoTrajectory();
                    traj->SetTorsoTrajectory(torso_traj);
                }
            }
        }
    }
}

} /* planner */

} /* dog_control */
