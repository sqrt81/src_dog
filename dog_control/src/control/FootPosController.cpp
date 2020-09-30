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

void FootPosController::ConnectPhysics(
        boost::shared_ptr<physics::DogPhysics> phys)
{
    phys_ptr_ = phys;
}

void FootPosController::ChangeFootControlMethod(FootConfigCRef config)
{
    assert(VALID_LEGNAME(config.foot_name));

    config_[config.foot_name] = config;
}

void FootPosController::SetFootState(FootStateCRef foot_state)
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
    boost::shared_ptr<physics::DogPhysics> phys = phys_ptr_.lock();

    assert(hw);
    assert(phys);

    // compute real stat according to joint states
    message::StampedJointState js = hw->GetJointState();

    for(unsigned int i = 0; i < 4; i++)
    {
        message::JointState3 one_leg_stat;
        FootState& real_stat = real_footstate_[i];

        std::copy(js.joint_state.begin() + i * 3,
                  js.joint_state.begin() + (i + 1) * 3,
                  one_leg_stat.begin());

        real_stat.pos = phys->ForwardKinematics(one_leg_stat);
        real_stat.vel = phys->ComputeFootVel(one_leg_stat);
    }


}

} /* control */

} /* dog_control */
