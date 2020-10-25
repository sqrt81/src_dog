#ifndef DOG_CONTROL_ESTIMATOR_EKFESTIMATOR_H
#define DOG_CONTROL_ESTIMATOR_EKFESTIMATOR_H

#include "EstimatorBase.h"

namespace dog_control
{

namespace estimator
{

class EKFEstimator : public EstimatorBase
{
public:
    EKFEstimator() = default;

    void Initialize(utils::ParamDictCRef dict) override;

    void Update() override;

private:
    double dt_;

    double var_a_;      // acceleration variance
    double var_w_;      // rotation vel variance
    double var_ba_;     // acc bias increament variance
    double var_bw_;     // rot_vel bias increasement variance
    double var_pi_;     // foot global pos variance
    double var_j_;      // joint pos variance
    double var_vj_;     // joint vel variance
    double var_s_;      // forward kinematics pos variance
    double var_vs_;     // forward kinematics vel variance

    Eigen::Vector3d gravity_;

    Eigen::MatrixXd Ak_; // jacobian matrix of X_k with respect to X_(k - 1)
    Eigen::MatrixXd Pk_; // noise covariance of the state
    Eigen::VectorXd Xk_; // state vector
    Eigen::MatrixXd Sk_; // noise covariance by observation
    Eigen::VectorXd Yk_; // observation (foot state error) vector
    Eigen::MatrixXd Hk_; // foot state jacobian with respect to X_k
    Eigen::MatrixXd Kk_; // kalman gain
    Eigen::MatrixXd Rk_; // temporary matrix for computing variance change
    Eigen::MatrixXd Qk_; // temporary matrix for observation basic variance
    Eigen::VectorXd Var_; // basic variance increment
};

} /* estimator */

} /* dog_control */

#endif // EKFESTIMATOR_H
