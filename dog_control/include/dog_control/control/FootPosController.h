#ifndef DOG_CONTROL_CONTROL_FOOTPOSCONTROLLER_H
#define DOG_CONTROL_CONTROL_FOOTPOSCONTROLLER_H

#include "dog_control/utils/ParamDict.h"

#include "dog_control/message/FootState.h"
#include "dog_control/message/MotorCommand.h"
#include "dog_control/utils/ClassDeclare.h"

#include <array>
#include <boost/weak_ptr.hpp>

namespace dog_control
{

namespace control
{

/**
 * @brief The FootPosController class
 * The foot position controller is used to "interpret"
 * from foot state to leg joints' state.
 * Which means, for upper level controllers,
 * e.g., MPC and TrajController, they only need to
 * care about foot movement.
 */
class FootPosController
{
protected:
    using FootState = message::FootState;
    using LegConfiguration = message::LegConfiguration;
    using FootStateCRef = message::FootStateCRef;
    using LegConfigCRef = message::LegConfigCRef;
    using JointForces = Eigen::Matrix<double, 12, 1>;

public:
    FootPosController();

    void Initialize(utils::ParamDictCRef dict);

    void ConnectHardware(boost::shared_ptr<hardware::HardwareBase> hw);
    void ConnectModel(boost::shared_ptr<physics::DogModel> model);

    void SetPipelineData(boost::shared_ptr<message::MotorCommand> cmd);

    /**
     * @brief ChangeFootControlMethod
     * Change configuration of joint kp, kd and IK.
     */
    void ChangeFootControlMethod(LegConfigCRef config);

    void SetFootStateCmd(FootStateCRef foot_state);

    void SetJointForceCmd(const JointForces& joint_forces);

    FootState GetFootState(message::LegName foot_name) const;

    void Update();

private:
    boost::weak_ptr<physics::DogModel> model_ptr_;
    boost::weak_ptr<hardware::HardwareBase> hw_ptr_;

    boost::shared_ptr<message::MotorCommand> cmd_;

    std::array<FootState, 4> cmd_footstate_;
    JointForces cmd_forces_;
    std::array<FootState, 4> real_footstate_;

    std::array<LegConfiguration, 4> config_;
};

} /* control */

} /* dog_control */

#endif /* DOG_CONTROL_CONTROL_FOOTPOSCONTROLLER_H */
