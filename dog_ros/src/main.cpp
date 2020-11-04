#include "dog_control/physics/DogModel.h"
#include "dog_control/control/TrajectoryController.h"
#include "dog_control/control/ConfSpaceTraj.h"
#include "dog_control/control/WholeBodyController.h"
#include "dog_control/control/FootPosController.h"
#include "dog_control/control/ModelPredictiveController.h"
#include "dog_ros/hardware/SimulatedHardware.h"
#include "dog_ros/estimator/CheaterEstimator.h"
#include "dog_control/estimator/EKFEstimator.h"
#include "dog_control/utils/Initializer.h"
#include "dog_control/utils/Math.h"
#include "dog_control/utils/MiniLog.h"
#include "dog_control/physics/EigenToolbox.h"
#include "dog_control/planner/HalfFlipPlanner.h"
#include "dog_ros/visualization/RvizVisualization.h"

#include <ros/ros.h>

using namespace dog_control;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "~");
    visualization::RvizVisualization vis;
    message::VisualData vis_data;
    vis.SetData(vis_data);

    std::string config_file;
    ros::param::get("~config_file", config_file);

    utils::ParamDict dict = utils::BuildParamDict(config_file);
    const double dt = ReadParOrDie(dict,
                                   PARAM_WITH_NS(control_period, control));
    boost::shared_ptr<message::MotorCommand> cmd
            (new message::MotorCommand());

    boost::shared_ptr<physics::DogModel> model(new physics::DogModel());
    boost::shared_ptr<hardware::ClockBase> clock(
                new hardware::SimulatedClock());
    boost::shared_ptr<hardware::HardwareBase> hw(
                new hardware::SimulatedHardware());
    boost::shared_ptr<estimator::EstimatorBase> estimator(
                new estimator::CheaterEstimator());
    boost::shared_ptr<estimator::EstimatorBase> ekf(
                new estimator::EKFEstimator());
    boost::shared_ptr<control::FootPosController> foot_ctrl(
                new control::FootPosController());
    boost::shared_ptr<control::WholeBodyController> wbc(
                new control::WholeBodyController());
    boost::shared_ptr<control::ModelPredictiveController> mpc(
                new control::ModelPredictiveController());
    boost::shared_ptr<control::TrajectoryController> traj(
                new control::TrajectoryController());

    boost::shared_ptr<planner::HalfFlipPlanner> half_flip(
                new planner::HalfFlipPlanner());

    model->Initialize(dict);
    hw->Initialize(dict);
    foot_ctrl->Initialize(dict);
    wbc->Initialize(dict);
    mpc->Initialize(dict);
    traj->Initialize(dict);
    ekf->Initialize(dict);
    half_flip->Initialize(dict);

    foot_ctrl->SetPipelineData(cmd);
    wbc->SetPipelineData(cmd);

    for (auto &item : *cmd)
    {
        item.x_desired = 0;
        item.v_desired = 0;
        item.kp = 0;
        item.kd = 0;
        item.torq = 0;
    }

    hw->PublishCommand(*cmd);

    model->ConnectHardware(hw);
    model->ConnectEstimator(estimator);
//    model->ConnectEstimator(ekf);
    estimator->ConnectHardware(hw);
    estimator->ConnectModel(model);
    ekf->ConnectModel(model);
    ekf->ConnectHardware(hw);
    foot_ctrl->ConnectModel(model);
    foot_ctrl->ConnectTraj(traj);
    mpc->ConnectModel(model);
    mpc->ConnectTraj(traj);
    mpc->ConnectClock(clock);
    wbc->ConnectTraj(traj);
    wbc->ConnectMPC(mpc);
    wbc->ConnectModel(model);
    wbc->ConnectClock(clock);
    traj->ConnectClock(clock);
    traj->ConnectModel(model);

    half_flip->ConnectTraj(traj);
    half_flip->ConnectFoot(foot_ctrl);
    half_flip->ConnectClock(clock);
    half_flip->ConnectModel(model);

    // spin to update hardware and time
    for (int i = 0; i < 10; i++)
    {
        ros::spinOnce();
        ros::Duration(0.001).sleep();
    }

    clock->Update();
    estimator->Update();
    model->Update();
    ekf->Update();

    estimator::EKFEstimator* est
            = reinterpret_cast<estimator::EKFEstimator*>(ekf.get());
    est->ResetTransform(Eigen::Vector3d(0.0, 0.0, 0.4),
                        Eigen::Quaterniond::Identity());
    model->Update();
    foot_ctrl->Update();
    traj->Update();

    message::LegConfiguration conf;
    conf.hip_outwards = true;
    conf.knee_outwards = true;
    conf.kd = 4;
    conf.kp = 30;

    for (int i = 0; i < 4; i++)
    {
        conf.foot_name = static_cast<message::LegName>(i);
        foot_ctrl->ChangeFootControlMethod(conf);
    }

    int iter = 0;

    {
        std::vector<message::StampedFloatingBaseState> torso_traj;

        message::StampedFloatingBaseState state;
        state.stamp = clock->Time();
        state.state.trans = {0, 0, 0.4};
        state.state.rot = Eigen::Quaterniond::Identity();
        state.state.linear_vel = Eigen::Vector3d::Zero();
        state.state.rot_vel = Eigen::Vector3d::Zero();
        torso_traj.push_back(state);

        state.stamp = clock->Time() + 0.5;
        state.state.trans = {0, 0, 0.15};
        state.state.rot = Eigen::Quaterniond::Identity();
        state.state.linear_vel = Eigen::Vector3d::Zero();
        state.state.rot_vel = Eigen::Vector3d::Zero();
        torso_traj.push_back(state);

        traj->SetTorsoTrajectory(torso_traj);
    }

    ros::Rate r(1. / dt);

    // temp standup function
    for (int i = 0; i < 500; i++)
    {
        if (!ros::ok())
            return 0;

        ros::spinOnce();

        clock->Update();
        traj->Update();
        estimator->Update();
        ekf->Update();
        model->Update();
        foot_ctrl->Update();
        mpc->Update();
        wbc->Update();
        hw->PublishCommand(*cmd);

        // update visualization data
        if (i % 20 == 0)
        {
            auto est = estimator->GetResult();
//            auto est = estimator->GetResult();
            vis_data.cur_pose.rot = est.orientation;
            vis_data.cur_pose.trans = est.position;
            vis_data.cur_pose.linear_vel = est.linear_vel;
            vis_data.cur_pose.rot_vel = est.rot_vel;

            traj->SampleTrajFromNow(4, 0.1, vis_data.torso_traj);
            Eigen::Vector3d pos;

            for (int j = 0; j < 4; j++)
            {
                pos = model->FootPos(message::LegName(j));
                vis_data.foot_local_pos[j]
                        = vis_data.cur_pose.rot.conjugate()
                        * (pos - vis_data.cur_pose.trans);
            }

//            std::array<Eigen::Vector3d, 4> force;
//            std::array<bool, 4> contact;
//            mpc->GetFeetForce(clock->Time(), force, contact);

//            for (int j = 0; j < 4; j++)
//                vis_data.foot_force[j]
//                        = vis_data.cur_pose.rot.conjugate() * force[j];
            vis_data.foot_force = est.foot_force;

            vis.Update();
        }

//        auto res = ekf->GetResult();
//        LOG(INFO) << "estimated pos " << res.position.transpose();

        r.sleep();
    }

//    constexpr double len = 0.05;
    constexpr int duration = 1000;
//    constexpr double t = duration / 1000.;

    iter = 0;

    while (ros::ok())
    {
        iter++;

        if (iter > duration)
            break;

//        if (iter == 400)
//        {
//            conf.kd = 4;
//            conf.kp = 30;

//            for (int i = 0; i < 4; i++)
//            {
//                conf.foot_name = static_cast<message::LegName>(i);
//                foot_ctrl->ChangeFootControlMethod(conf);
//            }
//        }

//        if (iter == 800)
//        {
//            conf.kd = 0.1;
//            conf.kp = 0.3;

//            for (int i = 0; i < 4; i++)
//            {
//                conf.foot_name = static_cast<message::LegName>(i);
//                foot_ctrl->ChangeFootControlMethod(conf);
//            }
//        }

        ros::spinOnce();

        half_flip->Update();

        clock->Update();
        traj->Update();
        estimator->Update();
        ekf->Update();
        model->Update();
        foot_ctrl->Update();
        mpc->Update();
        wbc->Update();
        hw->PublishCommand(*cmd);

        // update visualization data
        if (iter % 20 == 0)
        {
            auto est = estimator->GetResult();
//            auto est = estimator->GetResult();
            vis_data.cur_pose.rot = est.orientation;
            vis_data.cur_pose.trans = est.position;
            vis_data.cur_pose.linear_vel = est.linear_vel;
            vis_data.cur_pose.rot_vel = est.rot_vel;

            traj->SampleTrajFromNow(4, 0.1, vis_data.torso_traj);
            Eigen::Vector3d pos;

            for (int j = 0; j < 4; j++)
            {
                pos = model->FootPos(message::LegName(j));
                vis_data.foot_local_pos[j]
                        = vis_data.cur_pose.rot.conjugate()
                        * (pos - vis_data.cur_pose.trans);
            }

//            std::array<Eigen::Vector3d, 4> force;
//            std::array<bool, 4> contact;
//            mpc->GetFeetForce(clock->Time(), force, contact);

//            for (int j = 0; j < 4; j++)
//                vis_data.foot_force[j]
//                        = vis_data.cur_pose.rot.conjugate() * force[j];
            vis_data.foot_force = est.foot_force;

            vis.Update();
        }

//        auto res = ekf->GetResult();
//        LOG(INFO) << "estimated pos " << res.position.transpose();

        r.sleep();
    }

    iter = 0;

    while (ros::ok())
    {
        iter++;

//        if (iter % duration == 1)
//        {
//            std::vector<message::StampedFloatingBaseState> torso_traj;
//            const double sample_time = clock->Time();

//            {
//                message::StampedFloatingBaseState state;
//                state.state = model->TorsoState();
//                state.state.rot_vel.setZero();
//                state.state.linear_vel.setZero();
//                state.state.rot
//                        = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());

//                state.stamp = sample_time;
//                torso_traj.push_back(state);

//                state.stamp = sample_time + 5 * duration / 4 * 0.001;
//                torso_traj.push_back(state);
//            }

//            traj->SetTorsoTrajectory(torso_traj);
//            traj->UpdateFootPos();
//        }

        ros::spinOnce();

        clock->Update();
        traj->Update();
        estimator->Update();
        ekf->Update();
        model->Update();
        foot_ctrl->Update();
        mpc->Update();
        wbc->Update();
        hw->PublishCommand(*cmd);

        // update visualization data
        if (iter % 10 == 0)
        {
            auto est = estimator->GetResult();
            vis_data.cur_pose.rot = est.orientation;
            vis_data.cur_pose.trans = est.position;
            vis_data.cur_pose.linear_vel = est.linear_vel;
            vis_data.cur_pose.rot_vel = est.rot_vel;

            traj->SampleTrajFromNow(4, 0.1, vis_data.torso_traj);
            Eigen::Vector3d pos;

            for (int j = 0; j < 4; j++)
            {
                pos = model->FootPos(message::LegName(j));
                vis_data.foot_local_pos[j]
                        = vis_data.cur_pose.rot.conjugate()
                        * (pos - vis_data.cur_pose.trans);
            }

//            std::array<Eigen::Vector3d, 4> force;
//            std::array<bool, 4> contact;
//            mpc->GetFeetForce(clock->Time(), force, contact);

//            for (int j = 0; j < 4; j++)
//                vis_data.foot_force[j]
//                        = vis_data.cur_pose.rot.conjugate() * force[j];
            vis_data.foot_force = ekf->GetResult().foot_force;

            vis.Update();
        }

//        auto res = ekf->GetResult();
//        LOG(INFO) << "estimated pos " << res.position.transpose();

        r.sleep();
    }

    return 0;
}
