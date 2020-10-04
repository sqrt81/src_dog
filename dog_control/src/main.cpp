#include "dog_control/physics/DogModel.h"
#include "dog_control/hardware/SimulatedHardware.h"
#include "dog_control/estimator/CheaterEstimator.h"
#include "dog_control/control/FootPosController.h"
#include "dog_control/utils/Initializer.h"

#include <ros/ros.h>

#include <iostream>

using namespace dog_control;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "~");
    std::string config_file;
    ros::param::get("~config_file", config_file);

    utils::ParamDict dict = utils::BuildParamDict(config_file);

    boost::shared_ptr<physics::DogModel> model(new physics::DogModel());
    boost::shared_ptr<hardware::HardwareBase> hw(new hardware::SimulatedHardware());
    boost::shared_ptr<estimator::EstimatorBase> estimator(new estimator::CheaterEstimator());
    boost::shared_ptr<control::FootPosController> controller(new control::FootPosController());

    model->Initialize(dict);
    hw->Initialize(dict);
    controller->Initialize(dict);

    model->ConnectHardware(hw);
    model->ConnectEstimator(estimator);
    estimator->ConnectHardware(hw);
    estimator->ConnectModel(model);
    controller->ConnectHardware(hw);
    controller->ConnectModel(model);

    // spin once to update hardware
    ros::spinOnce();

    estimator->Update();
    model->Update();
    controller->Update();

    message::LegConfiguration conf;
    conf.kd = 1;
    conf.kp = 100;
    conf.joint_forces = Eigen::Vector3d::Zero();

    for(int i = 0; i < 4; i++)
    {
        conf.foot_name = static_cast<message::LegName>(i);
        controller->ChangeFootControlMethod(conf);
    }

    std::cout << "initialized." << std::endl;

    while(ros::ok())
    {
        for(int i = 0; i < 4; i++)
        {
            message::FootState fs;
            fs.foot_name = static_cast<message::LegName>(i);
            fs.pos = {0.25 * (i < 2      ? 1 : - 1),
                      0.10 * (i % 2 == 0 ? 1 : - 1),
                      0};
            fs.vel = Eigen::Vector3d::Zero();
            controller->SetFootStateCmd(fs);
        }

        ros::spinOnce();

        estimator->Update();
        model->Update();
        controller->Update();

        ros::Duration(0.001).sleep();
    }

    return 0;
}
