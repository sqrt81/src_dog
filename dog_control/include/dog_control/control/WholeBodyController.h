#ifndef DOG_CONTROL_CONTROL_WHOLEBODYCONTROLLER_H
#define DOG_CONTROL_CONTROL_WHOLEBODYCONTROLLER_H

#include "dog_control/physics/DogModel.h"
#include "dog_control/control/FootPosController.h"
#include "dog_control/message/MotorCommand.h"
#include "dog_control/utils/Initializer.h"

#include <Eigen/Eigen>
#include <array>
#include <boost/weak_ptr.hpp>

namespace dog_control
{

namespace control
{

class WholeBodyController
{
protected:
    using FBStateCRef = message::FBStateCRef;
    using FootStateCRef = message::FootStateCRef;
public:
    WholeBodyController();

    void Initialize(utils::ParamDictCRef dict);

    void ConnectModel(boost::shared_ptr<physics::DogModel> model);

    void SetPipelineData(boost::shared_ptr<message::MotorCommand> cmd);

    void SetTorsoMotionTask(FBStateCRef state_desired,
                            const Eigen::Vector3d &a_lin_desired,
                            const Eigen::Vector3d &a_rot_desired);

    void SetFootMotionTask(FootStateCRef state_desired,
                           const Eigen::Vector3d &a_desired);

    void SetRefFootForces(const std::array<Eigen::Vector3d, 4> &foot_forces,
                          const std::array<bool, 4> &foot_contact);

    void Update();
private:
    double kp_body_Cartesian_;
    double kd_body_Cartesian_;
    double kp_body_rotation_;
    double kd_body_rotation_;
    double kp_foot_Cartesian_;
    double kd_foot_Cartesian_;

    double q_f_; // loss factor for foot force difference
    double q_j_; // loss factor for joint acc difference
    double friction_; // ground friction factor

    boost::weak_ptr<physics::DogModel> model_ptr_;

    boost::shared_ptr<message::MotorCommand> cmd_;

    message::FloatingBaseState torso_state_task_;
    Eigen::Vector3d torso_acc_linear_task_;
    Eigen::Vector3d torso_acc_rot_task_;

    std::array<message::FootState, 4> foot_state_task_;
    std::array<Eigen::Vector3d, 4> foot_acc_task_;

    std::array<Eigen::Vector3d, 4> ref_force_;
    std::array<bool, 4> foot_contact_;

    // used for optimization
    Eigen::MatrixXf G_;
    Eigen::VectorXf g0_;
    Eigen::MatrixXf CE_;
    Eigen::VectorXf ce0_;
    Eigen::MatrixXf CI_;
    Eigen::VectorXf ci0_;
};

} /* control */

} /* dog_control */

#endif /* DOG_CONTROL_CONTROL_WHOLEBODYCONTROLLER_H */
