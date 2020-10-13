#include "dog_control/physics/DogModel.h"
#include "dog_control/hardware/SimulatedHardware.h"
#include "dog_control/estimator/CheaterEstimator.h"
#include "dog_control/control/WholeBodyController.h"
#include "dog_control/control/FootPosController.h"
#include "dog_control/utils/Initializer.h"
#include "dog_control/utils/Math.h"

#include <ros/ros.h>

#include <iostream>

using namespace dog_control;

int test_WBC_torso(int argc, char** argv)
{
    ros::init(argc, argv, "~");
    std::string config_file;
    ros::param::get("~config_file", config_file);

    utils::ParamDict dict = utils::BuildParamDict(config_file);
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

    model->Initialize(dict);
    hw->Initialize(dict);
    controller->Initialize(dict);
    wbc->Initialize(dict);

    controller->SetPipelineData(cmd);
    wbc->SetPipelineData(cmd);

    model->ConnectHardware(hw);
    model->ConnectEstimator(estimator);
    estimator->ConnectHardware(hw);
    estimator->ConnectModel(model);
    controller->ConnectHardware(hw);
    controller->ConnectModel(model);
    wbc->ConnectModel(model);

    // spin once to update hardware
    ros::spinOnce();

    estimator->Update();
    model->Update();
    controller->Update();

//    std::cout << std::endl;
    std::endl(std::cout);

    message::LegConfiguration conf;
    conf.kd = 2;
    conf.kp = 100;
    std::array<Eigen::Vector3d, 4> fake_ref_footforce;
    std::array<bool, 4> fake_foot_contact;
    ros::Rate r(1000);

    for (int i = 0; i < 4; i++)
    {
        conf.foot_name = static_cast<message::LegName>(i);
        controller->ChangeFootControlMethod(conf);

        fake_ref_footforce[i] = {0, 0, 25};
        fake_foot_contact[i] = true;
    }

    constexpr double M_PI_8 = M_PI / 8;

    // temp standup function
    for (int i = 0; i < 500; i++)
    {
        Eigen::AngleAxisd rot(i * M_PI_8 / 500, Eigen::Vector3d::UnitZ());
        rot = rot * Eigen::AngleAxisd(i * M_PI_8 / 500,
                                      Eigen::Vector3d::UnitX());
        for (int j = 0; j < 4; j++)
        {
            message::FootState fs;
            fs.foot_name = static_cast<message::LegName>(j);
            fs.pos = {0.283 * (j < 2      ? 1 : - 1),
                      0.118 * (j % 2 == 0 ? 1 : - 1),
                      - 0.4 + i * 0.1 / 500};
            fs.pos = rot.inverse() * fs.pos;
            fs.vel = {0, 0, 0.2};
            controller->SetFootStateCmd(fs);

            fs.pos = {0.283 * (j < 2      ? 1 : - 1),
                      0.118 * (j % 2 == 0 ? 1 : - 1),
                      0};
            fs.vel.z() = 0;
            wbc->SetFootMotionTask(fs, Eigen::Vector3d::Zero());
        }

        message::FloatingBaseState fbs;
        fbs.trans = {0, 0, 0.4 - i * 0.1 / 500};
        fbs.rot = rot;
        fbs.linear_vel = {0, 0, - 0.2};
        fbs.rot_vel.setZero();
        fbs.rot_vel.z() = M_PI_8 / 500 * 1000;

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
    constexpr double len = - 0.02;
    constexpr double duration = 400;
    constexpr double t = duration / 1000.;

    while(ros::ok())
    {
        iter++;

        Eigen::AngleAxisd rot(M_PI_8, Eigen::Vector3d::UnitZ());
        rot = rot * Eigen::AngleAxisd(M_PI_8, Eigen::Vector3d::UnitX());

        for (int i = 0; i < 4; i++)
        {
            message::FootState fs;
            fs.foot_name = static_cast<message::LegName>(i);
            fs.pos = {0.283 * (i < 2      ? 1 : - 1) + len
                      - len * cos(iter / duration * 2 * M_PI),
                      0.118 * (i % 2 == 0 ? 1 : - 1),
                      - 0.3};
            fs.pos = rot.inverse() * fs.pos;
            fs.vel = Eigen::Vector3d::Zero();
            fs.vel.x() = len * 2 * M_PI / t
                    * sin(iter / duration * 2 * M_PI);
            fs.vel = rot.inverse() * fs.vel;
            controller->SetFootStateCmd(fs);

            fs.pos.x() = 0.283 * (i < 2      ? 1 : - 1);
            fs.pos.y() = 0.118 * (i % 2 == 0 ? 1 : - 1);
//            fs.pos = rot.inverse() * fs.pos;
            fs.vel.setZero();
            wbc->SetFootMotionTask(fs, Eigen::Vector3d::Zero());
        }

        message::FloatingBaseState fbs;
        fbs.trans = {len * cos(iter / duration * 2 * M_PI) - len, 0, 0.3};
        fbs.rot = rot;
        fbs.linear_vel = {0, 0, 0};
        fbs.linear_vel.x() = - len * 2 * M_PI / t
                * sin(iter / duration * 2 * M_PI);
        fbs.linear_vel = rot.inverse() * fbs.linear_vel;
        fbs.rot_vel.setZero();
        Eigen::Vector3d torso_acc = {- len * utils::square(2 * M_PI / t)
                                      * cos(iter / duration * 2 * M_PI),
                                     0,
                                     0};

        wbc->SetTorsoMotionTask(fbs, torso_acc,
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

    return 0;
}
