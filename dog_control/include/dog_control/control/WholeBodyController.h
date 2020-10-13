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

/**
 * @brief The WholeBodyController class
 * The WBC takes torso and end effectors' desired position, velocity
 * and acceleration as its tasks.
 * A typical WBC should compute desired joints' position, velocity
 * and effort so as to achieve the above tasks.
 * However, the first two works are finished by FootPosController,
 * so here the WBC only needs to compute effort.
 */
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

    /**
     * @brief SetTorsoMotionTask
     * Set desired torso state and acceleration.
     * Note that desired accelerations are expressed in global frame.
     */
    void SetTorsoMotionTask(FBStateCRef state_desired,
                            const Eigen::Vector3d &a_lin_desired,
                            const Eigen::Vector3d &a_rot_desired);

    /**
     * @brief SetFootMotionTask
     * Set desired foot state and acceleration.
     * Note that desired accelerations are also expressed in global frame.
     */
    void SetFootMotionTask(FootStateCRef state_desired,
                           const Eigen::Vector3d &a_desired);

    /**
     * @brief SetRefFootForces
     * Set reference foot forces and foot contact state. The foot forces
     * decided by WBC will be close to the referece.
     * @param foot_forces       reference foot forces (may come from MPC)
     * @param foot_contact      foot contact state, true means in contact
     */
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
    double ground_friction_; // ground friction factor

    Eigen::Vector3d gravity_;

    boost::weak_ptr<physics::DogModel> model_ptr_;

    boost::shared_ptr<message::MotorCommand> cmd_;

    message::FloatingBaseState torso_state_task_;
    Eigen::Vector3d torso_acc_linear_task_;
    Eigen::Vector3d torso_acc_rot_task_;

    std::array<message::FootState, 4> foot_state_task_;
    std::array<Eigen::Vector3d, 4> foot_acc_task_;

    std::array<Eigen::Vector3d, 4> ref_force_;
    std::array<bool, 4> foot_contact_;

    Eigen::MatrixXd mass_;
    Eigen::MatrixXd inv_m_;
    Eigen::VectorXd force_bias_;
    Eigen::VectorXd vq_;
    Eigen::MatrixXd jacobian_;
    Eigen::MatrixXd null_space_;
    Eigen::MatrixXd null_space_i_;

    // used for optimization
    Eigen::MatrixXd G_;
    Eigen::VectorXd g0_;
    Eigen::MatrixXd CE_;
    Eigen::VectorXd ce0_;
    Eigen::MatrixXd CI_;
    Eigen::VectorXd ci0_;
};

} /* control */

} /* dog_control */

#endif /* DOG_CONTROL_CONTROL_WHOLEBODYCONTROLLER_H */
