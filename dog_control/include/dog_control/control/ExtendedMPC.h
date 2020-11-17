#ifndef EXTENDEDMPC_H
#define EXTENDEDMPC_H

#include "dog_control/control/MPCBase.h"

namespace dog_control
{

namespace control
{

class ExtendedMPC : public MPCBase
{
public:
    ExtendedMPC() = default;

    virtual void Initialize(utils::ParamDictCRef dict) override;

    virtual void Update() override;

    virtual void GetFeetForce(
            double t,
            std::array<Eigen::Vector3d, 4> &force,
            std::array<bool, 4> &contact) const override;

protected:
    boost::shared_ptr<physics::SimDogModel> pred_model_;

    // pred_horizon_ and pred_interval_ are deprecated.
    // Instead, ExtendedMPC use intervals of difference length.
    std::vector<double> pred_interval_seq_;

    // system matrixs
    // X_(k + 1) = A_k * X_k + B_k * F_k + C_k
    std::vector<Eigen::MatrixXd> A_;
    std::vector<Eigen::MatrixXd> B_;
    std::vector<Eigen::VectorXd> C_; // bias

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
