#include "dog_control/physics/FloatingBaseModel.h"
#include <iostream>

using namespace std;
using namespace dog_control::physics::spatial;

int main(int argc, char** argv)
{
    FloatingBaseModel model;
    NodeDescription node;
    node.inertia = BuildInertia(1., {0., 0., 0.},
                                Eigen::Matrix3d::Identity());
    node.joint_type = floating;
    std::cout << "adding link " << model.AddLink(node, 0) << std::endl;

    node.inertia = BuildInertia(1., {1., 0., 0.},
                                Eigen::Matrix3d::Identity() * 2);
    node.joint_axis << 1, 0, 0, 0, 0, 0;
    node.joint_type = revolute;
    node.X_parent = BuildTransform({1.5, 0, 1.},
                                   Eigen::Quaterniond(0.7071, 0, 0, 0.7071));

    std::cout << "adding link " << model.AddLink(node, 1) << std::endl;

    model.AddEndEffector(2, {0, 0, 1});

    FloatingBaseJointState stat;
    stat.base_rot = Eigen::Quaterniond(1, 0, 0, 0);
    stat.base_trans = Eigen::Vector3d(0, 0, 100);
    stat.base_linear_vel = Eigen::Vector3d(2, 2, 0);
    stat.base_rot_vel = Eigen::Vector3d(0, 0, 0);
    stat.q.push_back(0.);
    stat.dq.push_back(1.);

    model.SetJointMotionState(stat);
    model.ForwardKinematics();

    std::cout << "position \t" << model.EEPos(0).transpose() << std::endl;
    std::cout << "vel      \t" << model.EEVel(0).transpose() << std::endl;

//    model.DemoInfo();

    return 0;
}
