#include "dog_control/control/FootPosController.h"

#include <cassert>

namespace dog_control {

namespace control
{

FootPosController::FootPosController()
{
    for(unsigned int i = 0; i < 4; i++)
    {
        cmd_footstate_[i].foot_name = static_cast<message::LegName>(i);
        real_footstate_[i].foot_name = static_cast<message::LegName>(i);
        config_[i].foot_name = static_cast<message::LegName>(i);
    }
}

void FootPosController::Initialize(utils::ParamDictCRef dict)
{
    // looks like nothing needs to be done here
    (void) dict;
}

void FootPosController::ConnectHardware(
        boost::shared_ptr<hardware::HardwareBase> hw)
{
    hw_ptr_ = hw;
}

void FootPosController::ConnectModel(
        boost::shared_ptr<physics::DogModel> model)
{
    model_ptr_ = model;
}

void FootPosController::ChangeFootControlMethod(LegConfigCRef config)
{
    assert(VALID_LEGNAME(config.foot_name));

    config_[config.foot_name] = config;
}

void FootPosController::SetFootStateCmd(FootStateCRef foot_state)
{
    assert(VALID_LEGNAME(foot_state.foot_name));

    cmd_footstate_[foot_state.foot_name] = foot_state;
}

message::FootState FootPosController::GetFootState(
        message::LegName foot_name) const
{
    assert(VALID_LEGNAME(foot_name));

    return real_footstate_[foot_name];
}

void FootPosController::Update()
{
    boost::shared_ptr<hardware::HardwareBase> hw = hw_ptr_.lock();
    boost::shared_ptr<physics::DogModel> model = model_ptr_.lock();

    assert(hw);
    assert(model);

    // update foot position and velocity
    for(unsigned int i = 0; i < 4; i++)
    {
        FootState& real_stat = real_footstate_[i];

        real_stat.pos = model->FootPos(static_cast<message::LegName>(i));
        real_stat.vel = model->FootVel(static_cast<message::LegName>(i));
    }

    // apply commands
    message::MotorCommand cmd;

    for(unsigned int i = 0; i < 4; i++)
    {
        message::JointState3 leg_joint;
        model->InverseKinematics(static_cast<message::LegName>(i),
                                 cmd_footstate_[i].pos, leg_joint,
                                 (i >= 2), true);
        cmd[i * 3    ].x_desired = leg_joint[0].pos;
        cmd[i * 3 + 1].x_desired = leg_joint[1].pos;
        cmd[i * 3 + 2].x_desired = leg_joint[2].pos;

        const Eigen::Matrix3d jacob
                = model->JointJacob(static_cast<message::LegName>(i));
        Eigen::Vector3d vel = cmd_footstate_[i].vel;

        // avoid singularity
        if(jacob.determinant() > 1e-6)
            vel = jacob.inverse() * vel;
        else
            vel = (jacob + Eigen::Matrix3d::Identity() * 1e-2).inverse()
                    * vel;

        cmd[i * 3    ].v_desired = vel(0);
        cmd[i * 3 + 1].v_desired = vel(1);
        cmd[i * 3 + 2].v_desired = vel(2);

        LegConfigCRef config = config_[i];

        cmd[i * 3    ].kp = config.kp;
        cmd[i * 3 + 1].kp = config.kp;
        cmd[i * 3 + 2].kp = config.kp;
        cmd[i * 3    ].kd = config.kd;
        cmd[i * 3 + 1].kd = config.kd;
        cmd[i * 3 + 2].kd = config.kd;

        cmd[i * 3    ].torq = config.joint_forces(0);
        cmd[i * 3 + 1].torq = config.joint_forces(1);
        cmd[i * 3 + 2].torq = config.joint_forces(2);
    }

    hw->PublishCommand(cmd);
}

} /* control */

} /* dog_control */
