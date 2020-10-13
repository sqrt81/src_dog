#ifndef DOG_CONTROL_CONTROL_MODELPREDICTCONTROLLER_H
#define DOG_CONTROL_CONTROL_MODELPREDICTCONTROLLER_H

#include "dog_control/utils/ParamDict.h"
#include "dog_control/control/WholeBodyController.h"
#include "dog_control/physics/DogModel.h"
#include "dog_control/message/ModelJointState.h"

#include <Eigen/Eigen>
#include <array>
#include <vector>
#include <boost/weak_ptr.hpp>

namespace dog_control
{

namespace control
{

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

    void ConnectWBC(boost::shared_ptr<control::WholeBodyController> WBC);

    void ConnectModel(boost::shared_ptr<physics::DogModel> model);

    void SetDesiredTorsoTrajectory(const TorsoTraj &torso_traj);

    void SetCurTorsoPose(FBSCRef cur_state);

    void SetFeetPose(const FeetPosSeq& feet_pos,
                     const FeetContactSeq& feet_contact);

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
    double last_update_interval_;   // time elapsed after the last prediction
    double update_dt_;              // basic control period

    boost::weak_ptr<control::WholeBodyController> WBC_ptr_;
    boost::weak_ptr<physics::DogModel> model_ptr_;

    FBState cur_state_;
    TorsoTraj desired_traj_;
    FeetPosSeq feet_pos_seq_;
    FeetContactSeq feet_contact_seq_;

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

#endif /* DOG_CONTROL_CONTROL_MODELPREDICTCONTROLLER_H */
