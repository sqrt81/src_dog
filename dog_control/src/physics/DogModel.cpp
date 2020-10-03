#include "dog_control/physics/DogModel.h"

#include "dog_control/physics/SpatialToolbox.h"

#include <iostream>

namespace dog_control
{

namespace
{

#define PARAM_WITH_NS(param, ns) #ns "/" #param

inline double ReadParOrDie(utils::ParamDictCRef dict,
                           const std::string &param)
{
    const utils::ParamDict::const_iterator iter = dict.find(param);

    if(iter == dict.end())
    {
        std::cerr << "Param \"" << param << "\" not found." << std::endl;
        assert(false);
        exit(1);
    }
    else
        return iter->second;
}

} /* anonymous */

namespace physics
{

using namespace spatial;

spatial::FloatingBaseModel BuildDogModel(utils::ParamDictCRef dict)
{
    // temporaly store the calcuted node here
    std::array<NodeDescription, 12> storage_tmp;

    const double gear_ratio
            = ReadParOrDie(dict, PARAM_WITH_NS(gear_ratio, motor));
    const Eigen::Vector3d ee_local_pos(0, 0,
                ReadParOrDie(dict, PARAM_WITH_NS(shin_offset_z, physics)));
    Eigen::Matrix3d inertial = Eigen::Matrix3d::Zero();
    Eigen::Vector3d com = Eigen::Vector3d::Zero();
    Eigen::Vector3d tf_parent;
    double mass;

    // node of the hips
    inertial(0, 0) = ReadParOrDie(dict, PARAM_WITH_NS(I1xx, physics));
    inertial(1, 1) = ReadParOrDie(dict, PARAM_WITH_NS(I1yy, physics));
    inertial(2, 2) = ReadParOrDie(dict, PARAM_WITH_NS(I1zz, physics));
    com = Eigen::Vector3d(
                ReadParOrDie(dict, PARAM_WITH_NS(com_hip_x, physics)),
                ReadParOrDie(dict, PARAM_WITH_NS(com_hip_y, physics)),
                0);
    mass = ReadParOrDie(dict, PARAM_WITH_NS(hip_mass, physics));
    tf_parent = Eigen::Vector3d(
                ReadParOrDie(dict, PARAM_WITH_NS(hip_pos_x, physics)),
                ReadParOrDie(dict, PARAM_WITH_NS(hip_pos_y, physics)),
                0);

    for(int i = 0; i < 4; i++)
    {
        NodeDescription& nd = storage_tmp[i];
        const Eigen::Vector3d _com(com(0) * (i < 2      ? 1 : - 1),
                                   com(1) * (i % 2 == 0 ? 1 : - 1),
                                   com(2));
        const Eigen::Vector3d _tf(tf_parent(0) * (i < 2      ? 1 : - 1),
                                  tf_parent(1) * (i % 2 == 0 ? 1 : - 1),
                                  tf_parent(2));
        nd.inertia = BuildInertia(mass, _com, inertial);
        nd.joint_type = revolute;
        nd.joint_axis << (i % 2 == 0 ? 1 : -1), 0, 0, 0, 0, 0;
        nd.X_parent = BuildTransform(_tf, Eigen::Quaterniond(1, 0, 0, 0));
        nd.rotor_inertia = SMat::Zero(); // rotor is not modeled yet!
        nd.rotor_X_parent = nd.X_parent;
        nd.gear_ratio = gear_ratio;
    }

    // thigh
    inertial(0, 0) = ReadParOrDie(dict, PARAM_WITH_NS(I2xx, physics));
    inertial(1, 1) = ReadParOrDie(dict, PARAM_WITH_NS(I2yy, physics));
//    inertial(1, 2) = ReadParOrDie(dict, PARAM_WITH_NS(I2yz, physics));
//    inertial(2, 1) = inertial(1, 2);
    const double I2yz = ReadParOrDie(dict, PARAM_WITH_NS(I2yz, physics));
    inertial(2, 2) = ReadParOrDie(dict, PARAM_WITH_NS(I2zz, physics));
    com = Eigen::Vector3d(0,
                ReadParOrDie(dict, PARAM_WITH_NS(com_thigh_y, physics)),
                ReadParOrDie(dict, PARAM_WITH_NS(com_thigh_z, physics)));
    mass = ReadParOrDie(dict, PARAM_WITH_NS(thigh_mass, physics));
    tf_parent = Eigen::Vector3d(
                ReadParOrDie(dict, PARAM_WITH_NS(hip_len_x, physics)),
                ReadParOrDie(dict, PARAM_WITH_NS(hip_len_y, physics)),
                0);

    for(int i = 0; i < 4; i++)
    {
        NodeDescription& nd = storage_tmp[i + 4];
        const Eigen::Vector3d _com(com(0) * (i < 2      ? 1 : - 1),
                                   com(1) * (i % 2 == 0 ? 1 : - 1),
                                   com(2));
        const Eigen::Vector3d _tf(tf_parent(0) * (i < 2      ? 1 : - 1),
                                  tf_parent(1) * (i % 2 == 0 ? 1 : - 1),
                                  tf_parent(2));
        inertial(1, 2) = i % 2 == 0 ? I2yz : - I2yz;
        inertial(2, 1) = inertial(1, 2);
        nd.inertia = BuildInertia(mass, _com, inertial);
        nd.joint_type = revolute;
        nd.joint_axis << 0, (i < 2 ? 1 : -1), 0, 0, 0, 0;
        nd.X_parent = BuildTransform(_tf, Eigen::Quaterniond(1, 0, 0, 0));
        nd.rotor_inertia = SMat::Zero(); // rotor is not modeled yet!
        nd.rotor_X_parent = nd.X_parent;
        nd.gear_ratio = gear_ratio;
    }

    // shin
    inertial(0, 0) = ReadParOrDie(dict, PARAM_WITH_NS(I3xx, physics));
    inertial(1, 1) = ReadParOrDie(dict, PARAM_WITH_NS(I3yy, physics));
    inertial(1, 2) = 0;
    inertial(2, 1) = 0;
    inertial(2, 2) = ReadParOrDie(dict, PARAM_WITH_NS(I3zz, physics));
    com = Eigen::Vector3d(0, 0,
                ReadParOrDie(dict, PARAM_WITH_NS(com_shin_z, physics)));
    mass = ReadParOrDie(dict, PARAM_WITH_NS(shin_mass, physics));
    tf_parent = Eigen::Vector3d(0, 0,
                ReadParOrDie(dict, PARAM_WITH_NS(thigh_offset_z, physics)));

    for(int i = 0; i < 4; i++)
    {
        NodeDescription& nd = storage_tmp[i + 8];
        const Eigen::Vector3d _com(com(0) * (i < 2      ? 1 : - 1),
                                   com(1) * (i % 2 == 0 ? 1 : - 1),
                                   com(2));
        const Eigen::Vector3d _tf(tf_parent(0) * (i < 2      ? 1 : - 1),
                                  tf_parent(1) * (i % 2 == 0 ? 1 : - 1),
                                  tf_parent(2));
        nd.inertia = BuildInertia(mass, _com, inertial);
        nd.joint_type = revolute;
        nd.joint_axis << 0, (i < 2 ? 1 : -1), 0, 0, 0, 0;
        nd.X_parent = BuildTransform(_tf, Eigen::Quaterniond(1, 0, 0, 0));
        nd.rotor_inertia = SMat::Zero(); // rotor is not modeled yet!
        nd.rotor_X_parent = nd.X_parent;
        nd.gear_ratio = gear_ratio;
    }

    // at last, calculate node for torso
    NodeDescription torso;
    mass = ReadParOrDie(dict, PARAM_WITH_NS(torso_mass, physics));
    com = Eigen::Vector3d::Zero();
    inertial(0, 0) = ReadParOrDie(dict, PARAM_WITH_NS(I0xx, physics));
    inertial(1, 1) = ReadParOrDie(dict, PARAM_WITH_NS(I0yy, physics));
    inertial(2, 2) = ReadParOrDie(dict, PARAM_WITH_NS(I0zz, physics));
    torso.inertia = BuildInertia(mass, com, inertial);
    torso.joint_type = floating;

    FloatingBaseModel model;
    model.AddLink(torso, 0);

    for(int i = 0; i < 4; i++)
    {
        const Eigen::Vector3d _ee_local_pos(
                    ee_local_pos(0) * (i < 2      ? 1 : - 1),
                    ee_local_pos(1) * (i % 2 == 0 ? 1 : - 1),
                    ee_local_pos(2));
        int added_link = model.AddLink(storage_tmp[i], 1);
        added_link = model.AddLink(storage_tmp[4 + i], added_link);
        added_link = model.AddLink(storage_tmp[8 + i], added_link);
        model.AddEndEffector(added_link, _ee_local_pos);
    }

    return model;
}

void DogModel::Initialize(utils::ParamDictCRef dict)
{
    node_description_.resize(13);
    parent_.resize(13);
    ee_info_.resize(4);

    const double gear_ratio
            = ReadParOrDie(dict, PARAM_WITH_NS(gear_ratio, motor));
    const Eigen::Vector3d ee_local_pos(0, 0,
                ReadParOrDie(dict, PARAM_WITH_NS(shin_offset_z, physics)));
    Eigen::Matrix3d inertial = Eigen::Matrix3d::Zero();
    Eigen::Vector3d com = Eigen::Vector3d::Zero();
    Eigen::Vector3d tf_parent = Eigen::Vector3d::Zero();
    double mass;

    mass = ReadParOrDie(dict, PARAM_WITH_NS(torso_mass, physics));
    inertial(0, 0) = ReadParOrDie(dict, PARAM_WITH_NS(I0xx, physics));
    inertial(1, 1) = ReadParOrDie(dict, PARAM_WITH_NS(I0yy, physics));
    inertial(2, 2) = ReadParOrDie(dict, PARAM_WITH_NS(I0zz, physics));
    node_description_[0].inertia = BuildInertia(mass, com, inertial);
    node_description_[0].joint_type = floating;
    parent_[0] = - 1;

    // node of the hips
    inertial(0, 0) = ReadParOrDie(dict, PARAM_WITH_NS(I1xx, physics));
    inertial(1, 1) = ReadParOrDie(dict, PARAM_WITH_NS(I1yy, physics));
    inertial(2, 2) = ReadParOrDie(dict, PARAM_WITH_NS(I1zz, physics));
    com = Eigen::Vector3d(
                ReadParOrDie(dict, PARAM_WITH_NS(com_hip_x, physics)),
                ReadParOrDie(dict, PARAM_WITH_NS(com_hip_y, physics)),
                0);
    mass = ReadParOrDie(dict, PARAM_WITH_NS(hip_mass, physics));
    tf_parent = Eigen::Vector3d(
                ReadParOrDie(dict, PARAM_WITH_NS(hip_pos_x, physics)),
                ReadParOrDie(dict, PARAM_WITH_NS(hip_pos_y, physics)),
                0);

    for(int i = 0; i < 4; i++)
    {
        NodeDescription& nd = node_description_[i * 3 + 1];
        const Eigen::Vector3d _com(com(0) * (i < 2      ? 1 : - 1),
                                   com(1) * (i % 2 == 0 ? 1 : - 1),
                                   com(2));
        const Eigen::Vector3d _tf(tf_parent(0) * (i < 2      ? 1 : - 1),
                                  tf_parent(1) * (i % 2 == 0 ? 1 : - 1),
                                  tf_parent(2));
        nd.inertia = BuildInertia(mass, _com, inertial);
        nd.joint_type = revolute;
        nd.joint_axis << (i % 2 == 0 ? 1 : -1), 0, 0, 0, 0, 0;
        nd.X_parent = BuildTransform(_tf, Eigen::Quaterniond(1, 0, 0, 0));
        nd.rotor_inertia = SMat::Zero(); // rotor is not modeled yet!
        nd.rotor_X_parent = nd.X_parent;
        nd.gear_ratio = gear_ratio;

        parent_[i * 3 + 1] = 0;
    }

    // thigh
    inertial(0, 0) = ReadParOrDie(dict, PARAM_WITH_NS(I2xx, physics));
    inertial(1, 1) = ReadParOrDie(dict, PARAM_WITH_NS(I2yy, physics));
    const double I2yz = ReadParOrDie(dict, PARAM_WITH_NS(I2yz, physics));
    inertial(2, 2) = ReadParOrDie(dict, PARAM_WITH_NS(I2zz, physics));
    com = Eigen::Vector3d(0,
                ReadParOrDie(dict, PARAM_WITH_NS(com_thigh_y, physics)),
                ReadParOrDie(dict, PARAM_WITH_NS(com_thigh_z, physics)));
    mass = ReadParOrDie(dict, PARAM_WITH_NS(thigh_mass, physics));
    tf_parent = Eigen::Vector3d(
                ReadParOrDie(dict, PARAM_WITH_NS(hip_len_x, physics)),
                ReadParOrDie(dict, PARAM_WITH_NS(hip_len_y, physics)),
                0);

    for(int i = 0; i < 4; i++)
    {
        NodeDescription& nd = node_description_[i * 3 + 2];
        const Eigen::Vector3d _com(com(0) * (i < 2      ? 1 : - 1),
                                   com(1) * (i % 2 == 0 ? 1 : - 1),
                                   com(2));
        const Eigen::Vector3d _tf(tf_parent(0) * (i < 2      ? 1 : - 1),
                                  tf_parent(1) * (i % 2 == 0 ? 1 : - 1),
                                  tf_parent(2));
        inertial(1, 2) = i % 2 == 0 ? I2yz : - I2yz;
        inertial(2, 1) = inertial(1, 2);
        nd.inertia = BuildInertia(mass, _com, inertial);
        nd.joint_type = revolute;
        nd.joint_axis << 0, (i < 2 ? 1 : -1), 0, 0, 0, 0;
        nd.X_parent = BuildTransform(_tf, Eigen::Quaterniond(1, 0, 0, 0));
        nd.rotor_inertia = SMat::Zero(); // rotor is not modeled yet!
        nd.rotor_X_parent = nd.X_parent;
        nd.gear_ratio = gear_ratio;

        parent_[i * 3 + 2] = i * 3 + 1;
    }

    // shin
    inertial(0, 0) = ReadParOrDie(dict, PARAM_WITH_NS(I3xx, physics));
    inertial(1, 1) = ReadParOrDie(dict, PARAM_WITH_NS(I3yy, physics));
    inertial(1, 2) = 0;
    inertial(2, 1) = 0;
    inertial(2, 2) = ReadParOrDie(dict, PARAM_WITH_NS(I3zz, physics));
    com = Eigen::Vector3d(0, 0,
                ReadParOrDie(dict, PARAM_WITH_NS(com_shin_z, physics)));
    mass = ReadParOrDie(dict, PARAM_WITH_NS(shin_mass, physics));
    tf_parent = Eigen::Vector3d(0, 0,
                ReadParOrDie(dict, PARAM_WITH_NS(thigh_offset_z, physics)));

    for(int i = 0; i < 4; i++)
    {
        NodeDescription& nd = node_description_[i * 3 + 3];
        const Eigen::Vector3d _com(com(0) * (i < 2      ? 1 : - 1),
                                   com(1) * (i % 2 == 0 ? 1 : - 1),
                                   com(2));
        const Eigen::Vector3d _tf(tf_parent(0) * (i < 2      ? 1 : - 1),
                                  tf_parent(1) * (i % 2 == 0 ? 1 : - 1),
                                  tf_parent(2));
        nd.inertia = BuildInertia(mass, _com, inertial);
        nd.joint_type = revolute;
        nd.joint_axis << 0, (i < 2 ? 1 : -1), 0, 0, 0, 0;
        nd.X_parent = BuildTransform(_tf, Eigen::Quaterniond(1, 0, 0, 0));
        nd.rotor_inertia = SMat::Zero(); // rotor is not modeled yet!
        nd.rotor_X_parent = nd.X_parent;
        nd.gear_ratio = gear_ratio;

        parent_[i * 3 + 3] = i * 3 + 2;

        const Eigen::Vector3d _ee_local_pos(
                    ee_local_pos(0) * (i < 2      ? 1 : - 1),
                    ee_local_pos(1) * (i % 2 == 0 ? 1 : - 1),
                    ee_local_pos(2));
        ee_info_[i].ee_link_id = i * 3 + 3;
        ee_info_[i].ee_local_pos = _ee_local_pos;
    }
}

} /* physics */

} /* dog_control */
