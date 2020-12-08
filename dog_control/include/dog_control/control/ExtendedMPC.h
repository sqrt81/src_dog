#ifndef EXTENDEDMPC_H
#define EXTENDEDMPC_H

#include "dog_control/control/MPCBase.h"

#include "boost/thread.hpp"

namespace dog_control
{

namespace control
{

class ExtendedMPC : public MPCBase
{
public:
    ExtendedMPC() : update_running_(false) {};

    virtual void Initialize(utils::ParamDictCRef dict) override;

    virtual FBStateAcc GetTorsoState(double t) override;

    virtual void GetFeetForce(
            double t,
            std::array<Eigen::Vector3d, 4> &force,
            std::array<bool, 4> &contact) const override;

    virtual void Update() override;

protected:
    void Simulate();

    void SyncUpdate();

    boost::shared_ptr<physics::SimDogModel> pred_model_;

    bool update_running_;
    boost::thread update_thread_;

    // pred_horizon_ and pred_interval_ are deprecated.
    // Instead, ExtendedMPC use intervals of difference length.
    std::vector<double> pred_interval_seq_;

    double force_diff_w_;

    // system matrixs
    // X_(k + 1) = A_k * X_k + B_k * F_k + C_k
    std::vector<Eigen::MatrixXd> A_;
    std::vector<Eigen::MatrixXd> B_;
    std::vector<Eigen::VectorXd> C_; // bias
    std::vector<Eigen::VectorXd> X_;

    // desired joint vel at sample points
    std::vector<Eigen::VectorXd> Jvel_;

    Eigen::MatrixXd Aqp_Bqp_;
    Eigen::VectorXd Aqp_Cqp_;
    Eigen::VectorXd pred_force_;

    Eigen::MatrixXd G_;
    Eigen::VectorXd g0_;
//    Eigen::MatrixXd CI_;
//    Eigen::VectorXd ci0_;
//    Eigen::MatrixXd CE_;
//    Eigen::VectorXd ce0_;
};

} /* control */

} /* dog_control */

#endif // EXTENDEDMPC_H
