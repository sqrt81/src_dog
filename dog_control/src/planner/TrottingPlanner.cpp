#include "dog_control/planner/TrottingPlanner.h"

#include "dog_control/control/LocoSwingTraj.h"
#include "dog_control/control/TrajectoryController.h"
#include "dog_control/hardware/ClockBase.h"
#include "dog_control/estimator/EstimatorBase.h"
#include "dog_control/physics/DogModel.h"
#include "dog_control/utils/MiniLog.h"

namespace dog_control
{

namespace planner
{

void TrottingPlanner::Initialize(utils::ParamDictCRef dict)
{
    last_step_time_ = 0;
    foot_x_ = ReadParOrDie(dict, PARAM_WITH_NS(hip_pos_x, model));
    foot_y_ = ReadParOrDie(dict, PARAM_WITH_NS(hip_pos_y, model))
            + ReadParOrDie(dict, PARAM_WITH_NS(hip_len_y, model));
    kd_ = ReadParOrDie(dict, PARAM_WITH_NS(kd,
                                           planner/TROTTING));
    step_period_ = ReadParOrDie(dict, PARAM_WITH_NS(step_period,
                                                    planner/TROTTING));
    step_stance_period_ = ReadParOrDie(dict, PARAM_WITH_NS(step_stance_period,
                                                           planner/TROTTING));
    factor_ = 1 - ReadParOrDie(dict, PARAM_WITH_NS(smooth_factor,
                                                   planner/TROTTING));

    velocity_.x() = ReadParOrDie(dict, PARAM_WITH_NS(velocity_x,
                                                     planner/TROTTING));
    velocity_.y() = ReadParOrDie(dict, PARAM_WITH_NS(velocity_y,
                                                     planner/TROTTING));
    velocity_.z() = 0;

    torso_height_ = ReadParOrDie(dict, PARAM_WITH_NS(torso_height,
                                                     planner/TROTTING));
    step_height_ = ReadParOrDie(dict, PARAM_WITH_NS(step_height,
                                                    planner/TROTTING));

    filtered_vel_.setZero();
}

void TrottingPlanner::ConnectTraj(
        boost::shared_ptr<control::TrajectoryController> traj)
{
    traj_ptr_ = traj;
}

void TrottingPlanner::ConnectClock(
        boost::shared_ptr<hardware::ClockBase> clock)
{
    clock_ptr_ = clock;
}

void TrottingPlanner::ConnectEstimator(
        boost::shared_ptr<estimator::EstimatorBase> estimator)
{
    estimator_ = estimator;
}

void TrottingPlanner::ConnectModel(
        boost::shared_ptr<physics::DogModel> model)
{
    model_ = model;
}

void TrottingPlanner::Update()
{
    boost::shared_ptr<hardware::ClockBase> clock = clock_ptr_.lock();
    CHECK(clock) << "[TrottingPlanner] clock is not set!";
    const double cur_time = clock->Time();

    boost::shared_ptr<estimator::EstimatorBase> estimator
            = estimator_.lock();
    CHECK(estimator) << "[TrottingPlanner] estimator is not set!";

    boost::shared_ptr<control::TrajectoryController> traj
            = traj_ptr_.lock();
    CHECK(traj) << "[TrottingPlanner] traj is not set!";

    boost::shared_ptr<physics::DogModel> model
            = model_.lock();
    CHECK(model) << "[TrottingPlanner] model is not set!";

    if (cur_time > last_step_time_ + step_period_)
    {
        last_step_time_ = cur_time;
        filtered_vel_ += factor_ * (velocity_ - filtered_vel_);

        std::vector<message::StampedFloatingBaseState> torso_traj;
        const double end_time = cur_time + step_period_;
        const Eigen::Vector3d fl_local_pos
                = {foot_x_, foot_y_, - torso_height_};
        const Eigen::Vector3d fr_local_pos
                = {foot_x_, - foot_y_, - torso_height_};
        const Eigen::Vector3d bl_local_pos
                = {- foot_x_, foot_y_, - torso_height_};
        const Eigen::Vector3d br_local_pos
                = {- foot_x_, - foot_y_, - torso_height_};
        auto est = estimator->GetResult();

        {
            message::StampedFloatingBaseState state;
            state.state.trans = est.position;
            state.state.trans.z() = torso_height_;
            state.state.rot = Eigen::Quaterniond::Identity();
            state.state.linear_vel = filtered_vel_;
            state.state.rot_vel = Eigen::Vector3d::Zero();
            state.stamp = cur_time + 0.005;
            torso_traj.push_back(state);

            state.stamp = end_time;
            state.state.trans += filtered_vel_ * step_period_;
            torso_traj.push_back(state);
        }

        traj->SetTorsoTrajectory(torso_traj);

        // compute desired next zmp pos wrt next torso pos
        Eigen::Vector3d offset
                = kd_ * (est.orientation * est.linear_vel - filtered_vel_);
        offset.z() = 0;

        if (fl_br_swing_)
        {
            Eigen::Vector3d cur_pos;

            cur_pos = est.orientation.conjugate() *
                    (model->FootPos(message::FL) - est.position);
            boost::shared_ptr<control::FootSwingTrajBase> fl_traj(
                        new control::LocoSwingTraj(
                            cur_time + step_stance_period_,
                            end_time - step_stance_period_,
                            cur_pos, - filtered_vel_,
                            fl_local_pos + offset, - filtered_vel_,
                            Eigen::Vector3d(0, 0, step_height_)));
            traj->SetFootTrajectory(message::FL, fl_traj);

            cur_pos = est.orientation.conjugate() *
                    (model->FootPos(message::BR) - est.position);
            boost::shared_ptr<control::FootSwingTrajBase> br_traj(
                        new control::LocoSwingTraj(
                            cur_time + step_stance_period_,
                            end_time - step_stance_period_,
                            cur_pos, - filtered_vel_,
                            br_local_pos + offset, - filtered_vel_,
                            Eigen::Vector3d(0, 0, step_height_)));
            traj->SetFootTrajectory(message::BR, br_traj);
        }
        else
        {
            Eigen::Vector3d cur_pos;

            cur_pos = est.orientation.conjugate() *
                    (model->FootPos(message::FR) - est.position);
            boost::shared_ptr<control::FootSwingTrajBase> fr_traj(
                        new control::LocoSwingTraj(
                            cur_time + step_stance_period_,
                            end_time - step_stance_period_,
                            cur_pos, - filtered_vel_,
                            fr_local_pos + offset, - filtered_vel_,
                            Eigen::Vector3d(0, 0, step_height_)));
            traj->SetFootTrajectory(message::FR, fr_traj);

            cur_pos = est.orientation.conjugate() *
                    (model->FootPos(message::BL) - est.position);
            boost::shared_ptr<control::FootSwingTrajBase> bl_traj(
                        new control::LocoSwingTraj(
                            cur_time + step_stance_period_,
                            end_time - step_stance_period_,
                            cur_pos, - filtered_vel_,
                            bl_local_pos + offset, - filtered_vel_,
                            Eigen::Vector3d(0, 0, step_height_)));
            traj->SetFootTrajectory(message::BL, bl_traj);
        }

        fl_br_swing_ = !fl_br_swing_;
    }
}

} /* planner */

} /* dog_control */
