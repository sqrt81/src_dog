#include "dog_control/physics/DogModel.h"
#include "dog_control/hardware/SimulatedHardware.h"
#include "dog_control/estimator/CheaterEstimator.h"
#include "dog_control/control/WholeBodyController.h"
#include "dog_control/control/FootPosController.h"
#include "dog_control/control/ModelPredictController.h"
#include "dog_control/utils/Initializer.h"
#include "dog_control/utils/Math.h"
#include "dog_control/utils/MiniLog.h"

#include <ros/ros.h>

#include <iostream>

using namespace dog_control;

int test_MPC_ori(int argc, char** argv)
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
    boost::shared_ptr<hardware::HardwareBase> hw(
                new hardware::SimulatedHardware());
    boost::shared_ptr<estimator::EstimatorBase> estimator(
                new estimator::CheaterEstimator());
    boost::shared_ptr<control::FootPosController> controller(
                new control::FootPosController());
    boost::shared_ptr<control::WholeBodyController> wbc(
                new control::WholeBodyController());
    boost::shared_ptr<control::ModelPredictiveController> mpc(
                new control::ModelPredictiveController());

    model->Initialize(dict);
    hw->Initialize(dict);
    controller->Initialize(dict);
    wbc->Initialize(dict);
    mpc->Initialize(dict);

    controller->SetPipelineData(cmd);
    wbc->SetPipelineData(cmd);

    model->ConnectHardware(hw);
    model->ConnectEstimator(estimator);
    estimator->ConnectHardware(hw);
    estimator->ConnectModel(model);
    controller->ConnectHardware(hw);
    controller->ConnectModel(model);
    mpc->ConnectWBC(wbc);
    mpc->ConnectModel(model);
    wbc->ConnectModel(model);

    // spin once to update hardware
    ros::spinOnce();

    estimator->Update();
    model->Update();
    controller->Update();

//    std::cout << std::endl;
    std::endl(std::cout);

    message::LegConfiguration conf;
    conf.kd = 10;
    conf.kp = 100;
    std::array<Eigen::Vector3d, 4> fake_ref_footforce;
    std::array<bool, 4> fake_foot_contact;
    ros::Rate r(1. / dt);

    for (int i = 0; i < 4; i++)
    {
        conf.foot_name = static_cast<message::LegName>(i);
        controller->ChangeFootControlMethod(conf);

        fake_ref_footforce[i] = {0, 0, 25};
        fake_foot_contact[i] = true;
    }

    const double angle = M_PI_2 * 0.8;

    // temp standup function
    for (int i = 0; i < 500; i++)
    {
        Eigen::AngleAxisd rot(i * angle / 500,
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
            controller->SetFootStateCmd(fs);

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
        fbs.rot_vel.x() = angle / 500 * 1000;

        wbc->SetTorsoMotionTask(fbs, Eigen::Vector3d::Zero(),
                                Eigen::Vector3d::Zero());
        wbc->SetRefFootForces(fake_ref_footforce, fake_foot_contact);

        ros::spinOnce();

        estimator->Update();
        model->Update();
        controller->Update();
        wbc->Update();
        hw->PublishCommand(*cmd);

        r.sleep();
    }

    conf.kd = 1;
    conf.kp = 5;

    for (int i = 0; i < 4; i++)
    {
        conf.foot_name = static_cast<message::LegName>(i);
        controller->ChangeFootControlMethod(conf);
    }

    int iter = 0;
    constexpr double len = 0.2;
    constexpr double duration = 400;
    constexpr double t = duration / 1000.;

    // compute desired trajectory
    control::ModelPredictiveController::TorsoTraj torso_traj(6);
    control::ModelPredictiveController::FeetPosSeq fseq(6);
    control::ModelPredictiveController::FeetContactSeq fcseq(6);

    Eigen::AngleAxisd rot_base
            = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX());

    while(ros::ok())
    {
        iter++;

        Eigen::AngleAxisd rot = rot_base;
        rot = rot * Eigen::AngleAxisd(
                    len * sin(iter / duration * 2 * M_PI),
                    Eigen::Vector3d::UnitZ());

        if(iter % 10 == 1) // make sure mpc is inited in the first round
        {
            for(int i = 0; i < 6; i++)
            {
                message::FloatingBaseState& fbs = torso_traj[i];
                fbs.trans = {0, 0, 0.25};
                fbs.rot = rot_base * Eigen::AngleAxisd(
                            len * sin((iter + i) / duration * 2 * M_PI),
                            Eigen::Vector3d::UnitZ());
                fbs.linear_vel = {0, 0, 0};
                fbs.rot_vel.setZero();
                fbs.rot_vel.z() = len * 2 * M_PI / t
                        * cos((iter + i) / duration * 2 * M_PI);

                std::array<Eigen::Vector3d, 4>& fps = fseq[i];
                std::array<bool, 4>& fcs = fcseq[i];

                for(int j = 0; j < 4; j++)
                {
                    fps[j] = {0.283 * (j < 2      ? 1 : - 1),
                              0.118 * (j % 2 == 0 ? 1 : - 1),
                              0};
                    fcs[j] = true;
                }
            }

            mpc->SetDesiredTorsoTrajectory(torso_traj);
            mpc->SetFeetPose(fseq, fcseq);
        }

        message::FloatingBaseState fbs;
        fbs.trans = {0, 0, 0.25};
        fbs.rot = rot;
        fbs.linear_vel = {0, 0, 0};
        fbs.rot_vel.setZero();
        fbs.rot_vel.z() = len * 2 * M_PI / t
                * cos(iter / duration * 2 * M_PI);

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
            controller->SetFootStateCmd(fs);

            fs.pos = {0.283 * (i < 2      ? 1 : - 1),
                      0.118 * (i % 2 == 0 ? 1 : - 1),
                      0.};
//            fs.pos = rot.inverse() * fs.pos;
            fs.vel.setZero();
            wbc->SetFootMotionTask(fs, Eigen::Vector3d::Zero());
        }

        Eigen::Vector3d torso_acc = {0, 0,
                                     - len * utils::square(2 * M_PI / t)
                                      * sin(iter / duration * 2 * M_PI)};

        wbc->SetTorsoMotionTask(fbs, Eigen::Vector3d::Zero(),
                                rot * torso_acc);
//        wbc->SetRefFootForces(fake_ref_footforce, fake_foot_contact);

        ros::spinOnce();

        estimator->Update();
        model->Update();
        controller->Update();
        mpc->Update();
        wbc->Update();
        hw->PublishCommand(*cmd);

        r.sleep();
    }

    return 0;
}
