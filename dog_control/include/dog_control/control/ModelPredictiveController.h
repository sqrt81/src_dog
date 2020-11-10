#ifndef DOG_CONTROL_CONTROL_MODELPREDICTIVECONTROLLER_H
#define DOG_CONTROL_CONTROL_MODELPREDICTIVECONTROLLER_H

#include "dog_control/control/MPCBase.h"

#include <Eigen/Eigen>
#include <array>
#include <vector>
#include <boost/weak_ptr.hpp>

namespace dog_control
{

namespace control
{

/**
 * @brief The ModelPredictiveController class
 * Model Predictive Control is a type of
 * discrete-time optimal control.
 * To apply this control method, we need to make three approximations:
 * First, we model the entire robot as a single rigid-body
 * (keep only the torso and omit the effect of the legs).
 * Second, we omit all non-linear effects, supposing the sampling
 * interval is short enough.
 * Third, we suppose the real robot trajectory and the desired one
 * are close enough that the model calcuted with desired states
 * can be applied to the real one.
 * With the above approximations, the relationship between robot state
 * (translation, rotation and corresponding velocities)
 * and force applied to it (foot force and gravity)
 * is easily expressed as
 *   x_(k + 1) = A_k * x_k + B_k * f_k + g
 * where x_k, f_k denote robot state and all foot forces,
 * A_k, B_k are system matrix, and g denotes gravity impact.
 *
 * So, given a desired torso trajectory and current real state,
 * the model predictive controller will compute foot forces
 * to minimize the difference between real and desired torso trajectories.
 */
class ModelPredictiveController : public MPCBase
{
public:
    ModelPredictiveController() = default;

    virtual void Initialize(utils::ParamDictCRef dict) override;

//    /**
//     * @brief SetDesiredTorsoTrajectory
//     * Not used.
//     */
//    void SetDesiredTorsoTrajectory(const TorsoTraj &torso_traj);

//    /**
//     * @brief SetCurTorsoPose
//     * Note that when operating with other modules, this function is
//     * not used. Instead, current state and desired trajectory
//     * are obtained in Update().
//     */
//    void SetCurTorsoPose(FBSCRef cur_state);

//    void SetFeetPose(const FeetPosSeq &feet_pos,
//                     const FeetContactSeq &feet_contact);

    virtual void Update() override;

protected:
    double inv_mass_;
    Eigen::Matrix3d inv_inertia_;

    // system matrixs
    std::vector<Eigen::MatrixXd> A_;
    std::vector<Eigen::MatrixXd> B_;

    Eigen::MatrixXd Aqp_Bqp_;
    Eigen::VectorXd Aqp_C_X0_;
    Eigen::VectorXd pred_force_;

    // used for optimization
    Eigen::MatrixXd G_;
    Eigen::VectorXd g0_;
    Eigen::MatrixXd CI_;
    Eigen::VectorXd ci0_;
    Eigen::MatrixXd CE_;    // CE_ and ce0_ are not used here
    Eigen::VectorXd ce0_;
};

} /* control */

} /* dog_control */

#endif /* DOG_CONTROL_CONTROL_MODELPREDICTIVECONTROLLER_H */
