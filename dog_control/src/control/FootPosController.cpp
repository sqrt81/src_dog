#include "dog_control/control/FootPosController.h"

#include "dog_control/hardware/ClockBase.h"
#include "dog_control/control/TrajectoryController.h"
#include "dog_control/hardware/HardwareBase.h"
#include "dog_control/physics/DogModel.h"
#include "dog_control/utils/Initializer.h"
#include "dog_control/utils/Math.h"
#include "dog_control/utils/MiniLog.h"

namespace dog_control {

namespace control
{

FootPosController::FootPosController()
{
    for (unsigned int i = 0; i < 4; i++)
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

void FootPosController::ConnectModel(
        boost::shared_ptr<physics::DogModel> model)
{
    model_ptr_ = model;
}

void FootPosController::ConnectTraj(
        boost::shared_ptr<TrajectoryController> traj)
{
    traj_ptr_ = traj;
}

void FootPosController::SetPipelineData(
        boost::shared_ptr<message::MotorCommand> cmd)
{
    cmd_ = cmd;
}

void FootPosController::ChangeFootControlMethod(LegConfigCRef config)
{
    CHECK(VALID_LEGNAME(config.foot_name));

    config_[config.foot_name] = config;
}

void FootPosController::SetFootStateCmd(FootStateCRef foot_state)
{
    CHECK(VALID_LEGNAME(foot_state.foot_name));

    cmd_footstate_[foot_state.foot_name] = foot_state;
}

message::FootState FootPosController::GetFootState(
        message::LegName foot_name) const
{
    CHECK(VALID_LEGNAME(foot_name));

    return real_footstate_[foot_name];
}

void FootPosController::Update()
{
    boost::shared_ptr<physics::DogModel> model = model_ptr_.lock();
    CHECK(model) << "[FootPosController] Model is not set!";

    boost::shared_ptr<TrajectoryController> traj = traj_ptr_.lock();
    CHECK(traj) << "[FootPosController] Traj is not set!";

    // update foot position and velocity
    for (unsigned int i = 0; i < 4; i++)
    {
        FootState& real_stat = real_footstate_[i];

        real_stat.pos = model->FootPos(static_cast<message::LegName>(i));
        real_stat.vel = model->FootVel(static_cast<message::LegName>(i));
    }

    // apply commands
    for (unsigned int i = 0; i < 4; i++)
    {
        message::SingleMotorCommand* offset_cmd = cmd_->data() + i * 3;
        LegConfig& config = config_[i];

        traj->GetCurLocalFootState(static_cast<message::LegName>(i),
                                   cmd_footstate_[i].pos,
                                   cmd_footstate_[i].vel,
                                   config);

        const Eigen::Vector3d leg_joint =
                model->InverseKinematics(
                    static_cast<message::LegName>(i),
                    cmd_footstate_[i].pos,
                    config.knee_outwards, config.hip_outwards);
        offset_cmd[0].x_desired = leg_joint(0);
        offset_cmd[1].x_desired = leg_joint(1);
        offset_cmd[2].x_desired = leg_joint(2);

        const Eigen::Matrix3d jacob = model->ComputeJacobian(
                    static_cast<message::LegName>(i), leg_joint);
        Eigen::Vector3d vel = cmd_footstate_[i].vel;

        // avoid singularity
        if (utils::is_zero(jacob.determinant()))
        {
            vel = Eigen::Vector3d::Zero();
        }
        else
        {
            vel = jacob.inverse() * vel;
        }

        offset_cmd[0].v_desired = vel(0);
        offset_cmd[1].v_desired = vel(1);
        offset_cmd[2].v_desired = vel(2);

        offset_cmd[0].kp = config.kp;
        offset_cmd[1].kp = config.kp;
        offset_cmd[2].kp = config.kp;
        offset_cmd[0].kd = config.kd;
        offset_cmd[1].kd = config.kd;
        offset_cmd[2].kd = config.kd;
    }
}

} /* control */

} /* dog_control */
