#include "dog_control/physics/DogModel.h"
#include "dog_control/hardware/SimulatedHardware.h"
#include "dog_control/estimator/CheaterEstimator.h"
#include "dog_control/control/TrajectoryController.h"
#include "dog_control/control/WholeBodyController.h"
#include "dog_control/control/FootPosController.h"
#include "dog_control/control/ModelPredictiveController.h"
#include "dog_control/utils/Initializer.h"
#include "dog_control/utils/Math.h"
#include "dog_control/utils/MiniLog.h"
#include "dog_control/physics/EigenToolbox.h"

#include <ros/ros.h>
//#include <iostream>

using namespace dog_control;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "~");
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
    boost::shared_ptr<control::FootPosController> foot_ctrl(
                new control::FootPosController());
    boost::shared_ptr<control::WholeBodyController> wbc(
                new control::WholeBodyController());
    boost::shared_ptr<control::ModelPredictiveController> mpc(
                new control::ModelPredictiveController());
    boost::shared_ptr<control::TrajectoryController> traj(
                new control::TrajectoryController());

    model->Initialize(dict);
    hw->Initialize(dict);
    foot_ctrl->Initialize(dict);
    wbc->Initialize(dict);
    mpc->Initialize(dict);
    traj->Initialize(dict);

    foot_ctrl->SetPipelineData(cmd);
    wbc->SetPipelineData(cmd);

    model->ConnectHardware(hw);
    model->ConnectEstimator(estimator);
    estimator->ConnectHardware(hw);
    estimator->ConnectModel(model);
    foot_ctrl->ConnectHardware(hw);
    foot_ctrl->ConnectModel(model);
    mpc->ConnectModel(model);
    mpc->ConnectTraj(traj);
    mpc->ConnectClock(clock);
    wbc->ConnectMPC(mpc);
    wbc->ConnectModel(model);
    wbc->ConnectClock(clock);
    traj->ConnectClock(clock);

    // spin once to update hardware
    ros::spinOnce();

    // wait for time message
    ros::Duration(0.01).sleep();

    clock->Update();
    estimator->Update();
    model->Update();
    foot_ctrl->Update();
    traj->Update();

//    std::cout << std::endl;
    std::endl(std::cout);

    message::LegConfiguration conf;
    conf.hip_outwards = true;
    conf.knee_outwards = true;
    conf.kd = 0.1;
    conf.kp = 0.3;

    for (int i = 0; i < 4; i++)
    {
        conf.foot_name = static_cast<message::LegName>(i);
        foot_ctrl->ChangeFootControlMethod(conf);
    }

    const double angle = M_PI_2;
    int iter = 0;

    // compute desired trajectory
//    control::ModelPredictiveController::TorsoTraj torso_traj(6);
    control::ModelPredictiveController::FeetPosSeq fseq(6);
    control::ModelPredictiveController::FeetContactSeq fcseq(6);

    for (int j = 0; j < 6; j++)
    {
        std::array<Eigen::Vector3d, 4>& fps = fseq[j];
        std::array<bool, 4>& fcs = fcseq[j];

        for(int k = 0; k < 4; k++)
        {
            fps[k] = {0.283 * (k < 2      ? 1 : - 1),
                      0.118 * (k % 2 == 0 ? 1 : - 1),
                      0};
            fcs[k] = true;
        }
    }

    mpc->SetFeetPose(fseq, fcseq);

    {
        std::vector<message::StampedFloatingBaseState> torso_traj;

        message::StampedFloatingBaseState state;
        state.stamp = clock->Time();
        state.state.trans = {0, 0, 0.4};
        state.state.rot = Eigen::Quaterniond::Identity();
        state.state.linear_vel = Eigen::Vector3d::Zero();
        state.state.rot_vel = Eigen::Vector3d::Zero();
        torso_traj.push_back(state);

        state.stamp = clock->Time() + 0.1;
        state.state.trans = {0, 0, 0.35};
        state.state.linear_vel = {0, 0, - 0.5};
        state.state.rot_vel = {0, 0, 0};
        torso_traj.push_back(state);

        state.stamp = clock->Time() + 0.3;
        state.state.trans = {0, 0, 0.3};
        state.state.rot = Eigen::AngleAxisd(M_PI_4, Eigen::Vector3d::UnitX());
        state.state.linear_vel = {0, - 0.5 * sin(M_PI_4), - 0.5 * cos(M_PI_4)};
        state.state.rot_vel = {M_PI_4 / 0.4, 0, 0};
        torso_traj.push_back(state);

        state.stamp = clock->Time() + 0.5;
        state.state.trans = {0, 0, 0.25};
        state.state.rot = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX());
        state.state.linear_vel = Eigen::Vector3d::Zero();
        state.state.rot_vel = {0, 0, 0};
        torso_traj.push_back(state);

        traj->SetTorsoTrajectory(torso_traj);
    }

    ros::Rate r(1. / dt);

    // temp standup function
    for (int i = 0; i < 500; i++)
    {
        if (!ros::ok())
            return 0;
        iter++;
        Eigen::AngleAxisd rot((i < 100 ? 0 :  i - 100) * angle / 400,
                              Eigen::Vector3d::UnitX());

        for (int j = 0; j < 4; j++)
        {
            message::FootState fs;
            fs.foot_name = static_cast<message::LegName>(j);
            fs.pos = {0.283 * (j < 2      ? 1 : - 1),
                      0.118 * (j % 2 == 0 ? 1 : - 1),
                      - 0.4 + i * 0.15 / 500};
            fs.pos = rot.inverse() * fs.pos;
            fs.vel = {0, 0, 0.3};
            foot_ctrl->SetFootStateCmd(fs);

            fs.pos = {0.283 * (j < 2      ? 1 : - 1),
                      0.118 * (j % 2 == 0 ? 1 : - 1),
                      0};
            fs.vel.z() = 0;
            wbc->SetFootMotionTask(fs, Eigen::Vector3d::Zero());
        }

        message::FloatingBaseState fbs;
        fbs.trans = {0, 0, 0.4 - i * 0.15 / 500};
        fbs.rot = rot;
        fbs.linear_vel = {0, 0, - 0.3};
        fbs.rot_vel.setZero();

        if (i > 100)
            fbs.rot_vel.x() = angle / 500 * 1000;

        wbc->SetTorsoMotionTask(fbs, Eigen::Vector3d::Zero(),
                                Eigen::Vector3d::Zero());

        ros::spinOnce();

        clock->Update();
        traj->Update();
        estimator->Update();
        model->Update();
        foot_ctrl->Update();
        mpc->Update();
        wbc->Update();
        hw->PublishCommand(*cmd);

        r.sleep();
    }

    constexpr double len = 0.1;
    constexpr int duration = 400;
    constexpr double t = duration / 1000.;

    Eigen::AngleAxisd rot_base
            = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX());

    iter = 0;

    while (ros::ok())
    {
        iter++;

        if (iter % duration == 1)
        {
            std::vector<message::StampedFloatingBaseState> torso_traj;
            const double sample_time = clock->Time();

            for (int i = 0; i < 200; i++)
            {
                message::StampedFloatingBaseState state;
                const int offset = i * duration / 200;
                state.stamp = sample_time + offset * 0.001;
                state.state.trans = {0, 0, 0.25};
                state.state.rot = rot_base;
                state.state.rot = state.state.rot * Eigen::AngleAxisd(
                            len * sin(offset * 2 * M_PI / duration),
                            Eigen::Vector3d::UnitZ());
                state.state.linear_vel = Eigen::Vector3d::Zero();
                state.state.rot_vel = Eigen::Vector3d::Zero();
                state.state.rot_vel.z() = len * 2 * M_PI / t
                        * cos(offset * 2 * M_PI / duration);
                torso_traj.push_back(state);
            }

            traj->SetTorsoTrajectory(torso_traj);
        }

        Eigen::AngleAxisd rot = rot_base;
        rot = rot * Eigen::AngleAxisd(
                    len * sin(iter * 2 * M_PI / duration),
                    Eigen::Vector3d::UnitZ());

        message::FloatingBaseState fbs;
        fbs.trans = {0, 0, 0.25};
        fbs.rot = rot;
        fbs.linear_vel = {0, 0, 0};
        fbs.rot_vel.setZero();
        fbs.rot_vel.z() = len * 2 * M_PI / t
                * cos(iter * 2 * M_PI / duration);

        for (int i = 0; i < 4; i++)
        {
            message::FootState fs;
            fs.foot_name = static_cast<message::LegName>(i);
            fs.pos = {0.283 * (i < 2      ? 1 : - 1),
                      0.118 * (i % 2 == 0 ? 1 : - 1),
                      - 0.25};
            fs.vel = - fbs.rot_vel.cross(fs.pos); // global
            fs.vel = rot.inverse() * fs.vel; // local
            fs.pos = rot.inverse() * fs.pos;
            foot_ctrl->SetFootStateCmd(fs);

            fs.pos = {0.283 * (i < 2      ? 1 : - 1),
                      0.118 * (i % 2 == 0 ? 1 : - 1),
                      0.};
//            fs.pos = rot.inverse() * fs.pos;
            fs.vel.setZero();
            wbc->SetFootMotionTask(fs, Eigen::Vector3d::Zero());
        }

        Eigen::Vector3d torso_acc = {0, 0,
                                     - len * utils::square(2 * M_PI / t)
                                      * sin(iter * 2 * M_PI / duration)};

        wbc->SetTorsoMotionTask(fbs, Eigen::Vector3d::Zero(),
                                rot * torso_acc);

        ros::spinOnce();

        clock->Update();
        traj->Update();
        estimator->Update();
        model->Update();
        foot_ctrl->Update();
        mpc->Update();
        wbc->Update();
        hw->PublishCommand(*cmd);

        r.sleep();
    }

    return 0;
}
