#include "dog_control/utils/Initializer.h"
#include "dog_control/hardware/SimulatedHardware.h"

#include <ros/ros.h>
#include <iostream>

using namespace std;
using namespace dog_control;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dog_control");

    utils::ParamDict dict = utils::BuildParamDict(
                "/home/sqrt81/catkin_ws/src_dog/"
                "dog_control/config/dog_config.lua");

    boost::shared_ptr<hardware::HardwareBase> hardware_ptr(
        new hardware::SimulatedHardware());

    message::MotorCommand cmd;

    for(int i = 0; i < 12; i++)
    {
        cmd[i].kd = 10;
        cmd[i].kp = 50;
        cmd[i].x_desired = 0.0;
        cmd[i].v_desired = 0.0;
        cmd[i].torq = 0.;
    }

    cmd[0].x_desired = 1.0;

    hardware_ptr->Initialize(dict);

    while(ros::ok())
    {
        ros::spinOnce();
        hardware_ptr->PublishCommand(cmd);
        ros::Duration(0.001).sleep();
    }

//    hardware_ptr->Spin();

    return 0;
}
