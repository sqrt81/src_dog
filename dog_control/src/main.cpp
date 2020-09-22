#include "dog_control/utils/Initializer.h"
#include "dog_control/physics/DogPhysics.h"

#include <ros/ros.h>
#include <iostream>

using namespace std;
using namespace dog_control;

int main(int argc, char** argv)
{
    utils::ParamDict dict = utils::BuildParamDict(
                "/home/sqrt81/catkin_ws/src_dog/"
                "dog_control/config/dog_config.lua");
    physics::DogPhysics phys;
    phys.Initialize(dict);

    Eigen::Vector3d pos = Eigen::Vector3d::Random() * 0.1
             + Eigen::Vector3d(0.283, 0.118, - 0.3);
    Eigen::Vector3d vel = Eigen::Vector3d::Random();
    Eigen::Vector3d acc = Eigen::Vector3d::Random();
    Eigen::Vector3d gravity = Eigen::Vector3d::Random();
    physics::JointState3 js;
    phys.InverseKinematics(pos, js, true, true);

    Eigen::Matrix3d H, C, G;

    cout << "origin pos: " << pos.transpose() << endl;
    cout << "joint pos: " << js[0].pos << ' '
         << js[1].pos << ' ' << js[2].pos << endl;
    cout << "joint vel: " << vel.transpose() << endl;
    cout << "joint acc: " << acc.transpose() << endl;
    cout << "gravity  : " << gravity.transpose() << endl;
    js[0].vel = vel(0);
    js[1].vel = vel(1);
    js[2].vel = vel(2);
    js[0].acc = acc(0);
    js[1].acc = acc(1);
    js[2].acc = acc(2);

    cout << "force: " << phys.ComputeTorque(js, gravity).transpose() << endl;

    return 0;
}
