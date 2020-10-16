#ifndef DOG_CONTROL_CONTROL_MODELPREDICTIVECONTROLLER_H
#define DOG_CONTROL_CONTROL_MODELPREDICTIVECONTROLLER_H

#include "dog_control/utils/ClassDeclare.h"

#include "dog_control/message/ModelJointState.h"
#include "dog_control/utils/ParamDict.h"

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
class ModelPredictiveController
{
protected:
    using FBState = message::FloatingBaseState;
    using FBSCRef = message::FBStateCRef;

public:
    using TorsoTraj = std::vector<FBState>;
    using FeetPosSeq = std::vector<std::array<Eigen::Vector3d, 4>>;
    using FeetContactSeq = std::vector<std::array<bool, 4>>;

    ModelPredictiveController();

    void Initialize(utils::ParamDictCRef dict);

    void ConnectClock(boost::shared_ptr<hardware::ClockBase> clock);
    void ConnectTraj(boost::shared_ptr<control::TrajectoryController> traj);
    void ConnectModel(boost::shared_ptr<physics::DogModel> model);

    /**
     * @brief SetDesiredTorsoTrajectory
     * Not used.
     */
    void SetDesiredTorsoTrajectory(const TorsoTraj &torso_traj);

    /**
     * @brief SetCurTorsoPose
     * Note that when operating with other modules, this function is
     * not used. Instead, current state and desired trajectory
     * are obtained in Update().
     */
    void SetCurTorsoPose(FBSCRef cur_state);

    void SetFeetPose(const FeetPosSeq &feet_pos,
                     const FeetContactSeq &feet_contact);

    void GetFeetForce(double t,
                      std::array<Eigen::Vector3d, 4> &force,
                      std::array<bool, 4> &contact) const;

    void Update();

private:
    Eigen::VectorXd state_weight_;
    double force_weight_;

    double ground_friction_;
    double f_z_max_;
    double f_z_min_;    // minimal z force to keep foot in contact
    Eigen::Vector3d gravity_;
    double inv_mass_;
    Eigen::Matrix3d inv_inertia_;

    // pred_horizon_ means how many steps
    // MPC looks into the future in each prediction.
    // pred_interval_ is the time interval between two steps.
    // update_period_ is MPC update period,
    // i.e., the time interval between two predictions.
    unsigned int pred_horizon_;
    double pred_interval_;
    double update_period_;

    // Parameter
    double last_update_time_;       // time stamp of last prediction
    double update_dt_;              // basic control period

    boost::weak_ptr<control::TrajectoryController> traj_ptr_;
    boost::weak_ptr<hardware::ClockBase> clock_ptr_;
    boost::weak_ptr<physics::DogModel> model_ptr_;

    FBState cur_state_;
    TorsoTraj desired_traj_;
    FeetPosSeq feet_pos_seq_;
    FeetContactSeq contact_seq_;

    // system matrixs
    std::vector<Eigen::MatrixXd> A_;
    std::vector<Eigen::MatrixXd> B_;

    Eigen::MatrixXd Aqp_Bqp_;
    Eigen::VectorXd Aqp_C_X0_;
    Eigen::VectorXd pred_force_;

    // result storage
    std::vector<std::array<Eigen::Vector3d, 4>> foot_force_;

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
