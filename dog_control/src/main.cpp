#include "dog_control/physics/DogModel.h"
#include "dog_control/message/ModelJointState.h"
#include "dog_control/utils/Initializer.h"
#include "dog_control/physics/DogPhysics.h"

#include <ros/ros.h>

#include <iostream>

using namespace dog_control;
using namespace physics;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "~");
    std::string config_file;
    ros::param::get("~config_file", config_file);

    utils::ParamDict dict = utils::BuildParamDict(config_file);

    DogPhysics phys;
    phys.Initialize(dict);

    spatial::FloatingBaseModel dog_model = BuildDogModel(dict);
    message::FloatingBaseJointState js;
    js.base_trans = Eigen::Vector3d(0, 0, 0);
    js.base_rot = Eigen::Quaterniond(1, 0, 0, 0);
    js.base_linear_vel = Eigen::Vector3d(0, 0, 0);
    js.base_rot_vel = Eigen::Vector3d(0, 0, 10);
    js.q.resize(12, 0.);
    js.dq.resize(12, 0.);

    message::JointState3 js3;
//    phys.InverseKinematics(Eigen::Vector3d(0.25, 0.1, -0.3), js3, true, true);
//    phys.ComputeJointVel(Eigen::Vector3d(2, 3, 1.), js3);
    js3[0].pos = 0;
    js3[1].pos = 0;
    js3[2].pos = 0;
    js3[0].vel = 0;
    js3[1].vel = 0;
    js3[2].vel = 0;
    js.q[0] = js3[0].pos;
    js.q[1] = js3[1].pos;
    js.q[2] = js3[2].pos;
    js.q[3] = js3[0].pos;
    js.q[4] = js3[1].pos;
    js.q[5] = js3[2].pos;
    js.q[6] = js3[0].pos;
    js.q[7] = js3[1].pos;
    js.q[8] = js3[2].pos;
    js.q[9] = js3[0].pos;
    js.q[10] = js3[1].pos;
    js.q[11] = js3[2].pos;

    js.dq[0] = js3[0].vel;
    js.dq[1] = js3[1].vel;
    js.dq[2] = js3[2].vel;
    js.dq[3] = js3[0].vel;
    js.dq[4] = js3[1].vel;
    js.dq[5] = js3[2].vel;
    js.dq[6] = js3[0].vel;
    js.dq[7] = js3[1].vel;
    js.dq[8] = js3[2].vel;
    js.dq[9] = js3[0].vel;
    js.dq[10] = js3[1].vel;
    js.dq[11] = js3[2].vel;

    dog_model.SetJointMotionState(js);
    dog_model.ForwardKinematics();

    for(int i = 0; i < 4; i++)
    {
        std::cout << "foot pos " << i << " : "
                  << dog_model.EEPos(i).transpose() << std::endl
                  << "foot vel " << i << " : "
                  << dog_model.EEVel(i).transpose() << std::endl;
    }

    std::cout << "matrix by fb model: " << std::endl
              << dog_model.MassMatrix().block<6, 6>(0, 0) << std::endl;

    Eigen::Matrix3d H, C, G;
    phys.BuildDynamicsMatrixs(js3, H, C, G);

    std::cout << "matrix by physics: " << std::endl
              << H << std::endl;

    std::cout << "bias by fb model: " << std::endl
              << (dog_model.BiasForces()).transpose()
              << std::endl;
//    std::cout << "bias by physics: " << std::endl
//              << (C * Eigen::Vector3d(js3[0].vel, js3[1].vel, js3[2].vel)
//                 + G * Eigen::Vector3d(- 9.8, 0, 0))
//                 .transpose() << std::endl;

    return 0;
}
