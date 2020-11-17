#include "dog_control/estimator/EKFEstimator.h"

#include "dog_control/hardware/HardwareBase.h"
#include "dog_control/physics/DogModel.h"
#include "dog_control/physics/EigenToolbox.h"
#include "dog_control/utils/MiniLog.h"
#include "dog_control/utils/Math.h"

namespace dog_control
{

namespace estimator
{

namespace
{

constexpr int n_foot = 3 * 4;
constexpr int n_j = 12;

// number of states:
// 3 position, 3 velocity, 3 rotation,
// 3 position for each foot
// and 3-dimenson variable for accumulated pos / rot error
constexpr int n_var = 3 * 3 + n_foot + 3 * 2;

constexpr double inf_variance = 1e2;

class GammarFunc
{
public:
    GammarFunc(const Eigen::Vector3d &omega, double dt)
     : dt_(dt), dt_2_((1. / 2.) * utils::square(dt_)),
       dt_3_((1. / 3.) * dt_ * dt_2_),
       rot_vel_(omega.norm()), identity_(utils::is_zero(rot_vel_)),
       cw_(1. - cos(rot_vel_ * dt_)), sw_(sin(rot_vel_ * dt_))
    {
        if (!identity_)
        {
            Eigen::Vector3d axis = omega / rot_vel_;
            K1_ = physics::ToLowerMatrix(axis);
            K2_ = K1_ * K1_;
            inv_w_1_ = 1.0 / rot_vel_;
            inv_w_2_ = utils::square(inv_w_1_);
            inv_w_3_ = inv_w_1_ * inv_w_2_;
        }
    }

    Eigen::Matrix3d Rank(int r) const
    {
        if (identity_)
        {
            double val = 0.;

            switch (r)
            {
            case 0:
                val = 1.;
                break;
            case 1:
                val = dt_;
                break;
            case 2:
                val = dt_2_;
                break;
            case 3:
                val = dt_3_;
                break;
            default:
                LOG(ERROR) << "[GammarFunc] Rank " << r << " is invalid.";
                break;
            }

            return Eigen::Vector3d::Constant(val).asDiagonal();
        }

        switch (r)
        {
        case 0:
        {
            Eigen::Matrix3d res = sw_ * K1_ + cw_ * K2_;
            res.diagonal() += Eigen::Vector3d::Ones();

            return res;
        }
        case 1:
        {
            Eigen::Matrix3d res = inv_w_1_ * cw_ * K1_
                    + (dt_ - inv_w_1_ * sw_) * K2_;
            res.diagonal() += Eigen::Vector3d::Constant(dt_);

            return res;
        }
        case 2:
        {
            Eigen::Matrix3d res = (inv_w_1_ * dt_ - inv_w_2_ * sw_) * K1_
                    + (dt_2_ - inv_w_2_ * cw_) * K2_;
            res.diagonal() += Eigen::Vector3d::Constant(dt_2_);

            return res;
        }
        case 3:
        {
            Eigen::Matrix3d res
                    = (dt_2_ * inv_w_1_ - inv_w_3_ * cw_) * K1_
                    + (dt_3_ - inv_w_2_ * dt_ + inv_w_3_ * sw_) * K2_;
            res.diagonal() += Eigen::Vector3d::Constant(dt_3_);

            return res;
        }
        default:
            LOG(FATAL) << "[GammarFunc] Rank " << r << " is invalid.";
            return Eigen::Matrix3d::Zero();
        }
    }

private:
    double dt_;
    double dt_2_;
    double dt_3_;
    double rot_vel_;
    bool identity_;
    Eigen::Matrix3d K1_;
    Eigen::Matrix3d K2_;
    double cw_;
    double sw_;
    double inv_w_1_;
    double inv_w_2_;
    double inv_w_3_;
};

} /* anonymous */

void EKFEstimator::Initialize(utils::ParamDictCRef dict)
{
    dt_ = ReadParOrDie(dict, PARAM_WITH_NS(control_period, control));
    filter_decay_ = ReadParOrDie(dict, PARAM_WITH_NS(decay, estimator/EKF));

    var_a_ = ReadParOrDie(dict, PARAM_WITH_NS(var_a, estimator/EKF));
    var_w_ = ReadParOrDie(dict, PARAM_WITH_NS(var_w, estimator/EKF));
    var_ba_ = ReadParOrDie(dict, PARAM_WITH_NS(var_ba, estimator/EKF));
    var_bw_ = ReadParOrDie(dict, PARAM_WITH_NS(var_bw, estimator/EKF));
    var_pi_ = ReadParOrDie(dict, PARAM_WITH_NS(var_pi, estimator/EKF));
    var_j_ = ReadParOrDie(dict, PARAM_WITH_NS(var_j, estimator/EKF));
    var_vj_ = ReadParOrDie(dict, PARAM_WITH_NS(var_vj, estimator/EKF));
    var_s_ = ReadParOrDie(dict, PARAM_WITH_NS(var_s, estimator/EKF));
    var_vs_ = ReadParOrDie(dict, PARAM_WITH_NS(var_vs, estimator/EKF));

    gravity_ = Eigen::Vector3d(
                ReadParOrDie(dict, PARAM_WITH_NS(gravity_x, physics)),
                ReadParOrDie(dict, PARAM_WITH_NS(gravity_y, physics)),
                ReadParOrDie(dict, PARAM_WITH_NS(gravity_z, physics)));

    Ak_ = Eigen::MatrixXd::Identity(n_var, n_var);
    Pk_ = Eigen::MatrixXd::Zero(n_var, n_var);
    Xk_ = Eigen::VectorXd::Zero(n_var);
    Sk_ = Eigen::MatrixXd::Zero(n_foot * 2, n_foot * 2);
    Yk_ = Eigen::VectorXd::Zero(n_foot * 2);
    Hk_ = Eigen::MatrixXd::Zero(n_foot * 2, n_var); // foot pos and vel
    Kk_ = Eigen::MatrixXd::Zero(n_var, n_foot * 2);
    Rk_ = Eigen::MatrixXd::Zero(n_var, n_var);
    Qk_ = Eigen::MatrixXd::Zero(n_foot * 2, n_foot * 2);
    Var_ = Eigen::VectorXd::Zero(n_var);

    Ak_.block<3, 3>(0, 3).diagonal().setConstant(dt_);

    const double dt_2 = (1. / 2.) * utils::square(dt_);
    const double dt_3 = (2. / 3.) * dt_ * dt_2;
    const double dt_4 = (3. / 8.) * dt_ * dt_3;
    const double dt_5 = (8. / 20.) * dt_ * dt_4;

    Var_.segment<3>(0).setConstant(dt_3 * var_a_ + dt_5 * var_ba_);
    Var_.segment<3>(3).setConstant(dt_ * var_a_ + dt_3 * var_ba_);
    Var_.segment<3>(6).setConstant(dt_ * var_w_);
    Var_.segment<n_foot>(9).setConstant(dt_ * var_pi_);
    Var_.segment<3>(21).setConstant(dt_ * var_ba_);
    Var_.segment<3>(24).setConstant(dt_ * var_bw_);

    res_.position.setZero();
    res_.orientation.setIdentity();
    res_.linear_vel.setZero();
    res_.rot_vel.setZero();

    torq_ = Eigen::VectorXd::Zero(n_j);
    vj_ = Eigen::VectorXd::Zero(6 + n_j);
    dvj_ = Eigen::VectorXd::Zero(6 + n_j);

    // Init variance for foot with a large value since
    // we do not know their place at first
    Pk_.diagonal().segment<n_foot>(9).setConstant(inf_variance);
}

void EKFEstimator::Update()
{
    // Obtain sensor data
    boost::shared_ptr<hardware::HardwareBase> hw = hw_ptr_.lock();
    CHECK(hw) << "[EKFEstimator] hardware is not connected!";

    boost::shared_ptr<physics::DogModel> model = model_ptr_.lock();
    CHECK(model) << "[EKFEstimator] model is not connected!";

    message::StampedImu imu = hw->GetImuData();
    message::StampedJointState js = hw->GetJointState();

    Eigen::Vector3d ft_pos[4];
    Eigen::Vector3d ft_vel[4];
    Eigen::Matrix3d jacob[4];
    Eigen::Matrix3d djacob[4];
    Eigen::Matrix<double, n_j, 1> cur_torq;

    for (int i = 0; i < 4; i++)
    {
        Eigen::Vector3d joint_pos(
                    js.joint_state[i * 3    ].pos,
                    js.joint_state[i * 3 + 1].pos,
                    js.joint_state[i * 3 + 2].pos);
        Eigen::Vector3d joint_vel(
                    js.joint_state[i * 3    ].vel,
                    js.joint_state[i * 3 + 1].vel,
                    js.joint_state[i * 3 + 2].vel);

        ft_pos[i] = model->ComputeLocalPos(static_cast<message::LegName>(i),
                                           joint_pos);
        jacob[i] = model->ComputeJacobian(static_cast<message::LegName>(i),
                                          joint_pos);
        djacob[i] = model->ComputeDJacobian(static_cast<message::LegName>(i),
                                           joint_pos, joint_vel);
        ft_vel[i] = jacob[i] * joint_vel;
        dvj_.segment<3>(6 + i * 3)
                = (joint_vel - vj_.segment<3>(6 + i * 3));
        cur_torq(i * 3    ) = - js.joint_state[i * 3    ].eff;
        cur_torq(i * 3 + 1) = - js.joint_state[i * 3 + 1].eff;
        cur_torq(i * 3 + 2) = - js.joint_state[i * 3 + 2].eff;
    }

    /* ------------Estimate foot force---------------- */
    cur_torq += model->MassMatrix().bottomRows<n_j>() * (dvj_ / dt_)
             + model->BiasForces().tail<n_j>()
             - model->Friction();

    torq_ = torq_ * (1. - filter_decay_) + cur_torq * filter_decay_;
    vj_.tail<n_j>() += dvj_.tail<n_j>();

    for (int i = 0; i < 4; i++)
    {
        // torq_ext = J.transpose * F_ext
        Eigen::Matrix3d inv_j;
        bool invertable;
        jacob[i].computeInverseWithCheck(inv_j, invertable, utils::precision);

        if (invertable)
        {
            res_.foot_force[i] = inv_j.transpose() * torq_.segment<3>(i * 3);
            res_.foot_contact[i] = res_.foot_force[i].norm() > 5.;
        }
        else
        {
            res_.foot_force[i].setZero();
            // ignore the foot because its velocity is unobservable
            res_.foot_contact[i] = false;
        }
    }

    /* ------------Estimate robot state--------------- */
    // acceleration is modeled to have a gaussian noise Na and
    // a bias Ba whose increment is gaussian noise, i.e.,
    // a_obs = a + Ba + Na
    // Na ~ N(0, var_a_)
    // dBa/dt ~ N(0, var_ba_)
    // and Ba is estimated by Xk_.segment<3>(21).
    // The rotation velocity is modeled in the same way.
    const Eigen::Vector3d acc_local
            = imu.imu_data.acceleration - Xk_.segment<3>(21);
    const Eigen::Vector3d rot_vel
            = imu.imu_data.angular_speed - Xk_.segment<3>(24);
    const Eigen::Vector3d acc_global
            = res_.orientation * acc_local + gravity_;
    const double dt_2 = (1. / 2.) * utils::square(dt_);
    const double dt_3 = (2. / 3.) * dt_ * dt_2;
    const double dt_4 = (3. / 8.) * dt_ * dt_3;
    const GammarFunc gammar(rot_vel, dt_);

    // Step 1: Update state according to system model
    // global_pos
    Xk_.segment<3>(0) += dt_ * Xk_.segment<3>(3) + dt_2 * acc_global;
    // global_vel
    Xk_.segment<3>(3) += dt_ * acc_global;
    // orientation, it is tracked by another variable,
    // and Xk_ tracks its correction term
    const Eigen::Quaterniond orientation
            = res_.orientation * physics::SO3ToQuat(rot_vel * dt_);
    Xk_.segment<3>(6).setZero();
    // the foot positions should be kept same since they stay on the ground
    // and the accumulated errors are also kept same

    // prepare some constants
    const Eigen::Matrix3d Ck_pre = res_.orientation.toRotationMatrix();
    const Eigen::Matrix3d Ck = orientation.toRotationMatrix();
    const Eigen::Matrix3d fk = physics::ToLowerMatrix(acc_local);
    const Eigen::Matrix3d wk = physics::ToLowerMatrix(rot_vel);
    const Eigen::Matrix3d wk_Ck = wk * Ck.transpose();
    const Eigen::Vector3d v_local
            = orientation.conjugate() * Xk_.segment<3>(3);
    const Eigen::Matrix3d Ck_vk = physics::ToLowerMatrix(v_local);
    const Eigen::Matrix3d g_2 = gammar.Rank(2);
    const Eigen::Matrix3d g_3 = gammar.Rank(3);

    // Step 2: Compute observations.
    Eigen::Vector3d foot_local_pos[4];

    for (int i = 0; i < 4; i++)
    {
        foot_local_pos[i] = orientation.conjugate()
                * (Xk_.segment<3>(i * 3 + 9) - Xk_.segment<3>(0));

        Yk_.segment<3>(i * 3) = ft_pos[i] - foot_local_pos[i];
        Yk_.segment<3>(i * 3 + n_foot) = ft_vel[i] + v_local
                + rot_vel.cross(foot_local_pos[i]);
    }

    // Step 3: Build jacobian matrix Ak_, Hk_
    Ak_.block<3, 3>(6, 6) = gammar.Rank(0).transpose();
    Ak_.block<3, 3>(3, 21) = - dt_ * Ck_pre;
    Ak_.block<3, 3>(0, 21) = 0.5 * dt_ * Ak_.block<3, 3>(3, 21);
    // = - dt_2 * Ck;
    Ak_.block<3, 3>(3, 6) = Ak_.block<3, 3>(3, 21) * fk;
    // = - dt * Ck * fk
    Ak_.block<3, 3>(0, 6) = 0.5 * dt_ * Ak_.block<3, 3>(3, 6);
    // = - dt_2 * Ck * fk
    Ak_.block<3, 3>(6, 24) = - gammar.Rank(1).transpose();

    for (int i = 0; i < 4; i++)
    {
        Hk_.block<3, 3>(i * 3,          0) = - Ck.transpose();
        Hk_.block<3, 3>(i * 3 + n_foot, 0) = wk_Ck;
        Hk_.block<3, 3>(i * 3 + n_foot, 3) = - Ck.transpose();
        Hk_.block<3, 3>(i * 3,          6) = physics::ToLowerMatrix(
                    foot_local_pos[i]);
        Hk_.block<3, 3>(i * 3 + n_foot, 6)
                = - wk * Hk_.block<3, 3>(i * 3, 6) - Ck_vk;
        Hk_.block<3, 3>(i * 3,          i * 3 + 9) = Ck.transpose();
        Hk_.block<3, 3>(i * 3 + n_foot, i * 3 + 9) = - wk_Ck;
        Hk_.block<3, 3>(i * 3 + n_foot, 24) = - Hk_.block<3, 3>(i * 3, 6);
    }

    for (int i = 0; i < 4; i++)
    {
    }

    // Step 4: Compute state and observation covariance matrix.
    Pk_ = Ak_ * Pk_ * Ak_.transpose();

    Pk_.diagonal() += Var_;
    Pk_.block<3, 3>(3, 0).diagonal().array()
            += dt_2 * var_a_ + dt_4 * var_ba_;
    Pk_.block<3, 3>(0, 3).diagonal().array()
            += dt_2 * var_a_ + dt_4 * var_ba_;
    Pk_.block<3, 3>(6, 6) += var_bw_ * (g_3 + g_3.transpose());
    Pk_.block<3, 3>(6, 24) -= var_bw_ * g_2.transpose();
    Pk_.block<3, 3>(24, 6) -= var_bw_ * g_2;
    Pk_.block<3, 3>(0, 21) -= 0.5 * dt_3 * var_ba_ * Ck;
    Pk_.block<3, 3>(3, 21) -= dt_2 * var_ba_ * Ck;
    Pk_.block<3, 3>(21, 0) -= 0.5 * dt_3 * var_ba_ * Ck.transpose();
    Pk_.block<3, 3>(21, 3) -= dt_2 * var_ba_ * Ck.transpose();

    for (int i = 0; i < 4; i++)
    {
        Qk_.block<3, 3>(i * 3, i * 3)
                = var_j_ * jacob[i] * jacob[i].transpose();
        Qk_.block<3, 3>(i * 3 + n_foot, i * 3 + n_foot)
                = var_vj_ * jacob[i] * jacob[i].transpose()
                + var_j_ * djacob[i] * djacob[i].transpose();
        Qk_.block<3, 3>(i * 3, i * 3 + n_foot)
                = var_j_ * jacob[i] * djacob[i].transpose();
        Qk_.block<3, 3>(i * 3 + n_foot, i * 3)
                = Qk_.block<3, 3>(i * 3, i * 3 + n_foot).transpose();

        if (!res_.foot_contact[i])
        {
            // If a foot is not in contact, the algorithm should ignore
            // its position and velocity.
            // To achieve this goal, we set the variance to infinity.
            Pk_.diagonal().segment<3>(9 + i * 3).setConstant(inf_variance);
            Qk_.diagonal().segment<3>(i * 3 + n_foot)
                    .setConstant(inf_variance);
        }
    }

    Qk_.diagonal().head<n_foot>().array() += var_s_;
    Qk_.diagonal().tail<n_foot>().array() += var_vs_;

    Sk_.noalias() = Hk_ * Pk_ * Hk_.transpose() + Qk_;

    // Step 5: Run Kalman filter.
    Kk_.noalias() = Pk_ * Hk_.transpose() * Sk_.inverse();
    Xk_ += Kk_ * Yk_;
    Rk_.noalias() = - Kk_ * Hk_;
    Rk_.diagonal().array() += 1.;
    Pk_.applyOnTheLeft(Rk_);

    // Step 6: Dump the result.
    res_.position = Xk_.segment<3>(0);
    res_.orientation = orientation * physics::SO3ToQuat(Xk_.segment<3>(6));
    res_.orientation.normalize();
    res_.linear_vel = res_.orientation.conjugate() * Xk_.segment<3>(3);
    res_.rot_vel = imu.imu_data.angular_speed - Xk_.segment<3>(24);
    res_.linear_acc = imu.imu_data.acceleration - Xk_.segment<3>(21)
            + res_.orientation.conjugate() * gravity_;

    // update torso vel difference.
    // be aware they are expressed in floating frame
    dvj_.segment<3>(0) = res_.rot_vel - vj_.segment<3>(0);
    dvj_.segment<3>(3) = res_.orientation.conjugate()
            * (Xk_.segment<3>(3) - vj_.segment<3>(3) - gravity_ * dt_);
    // update torso velocity. use global linear velocity for convenience
    vj_.segment<3>(0) = res_.rot_vel;
    vj_.segment<3>(3) = Xk_.segment<3>(3);
}

void EKFEstimator::ResetTransform(const Eigen::Vector3d &trans,
                                  const Eigen::Quaterniond &rot)
{
    const Eigen::Quaterniond R = rot * res_.orientation.conjugate();

    Xk_.segment<3>(0) = trans;
    Xk_.segment<3>(3) = R * Xk_.segment<3>(3);
    Xk_.segment<3>(9) = trans + R * (Xk_.segment<3>(9) - res_.position);
    Xk_.segment<3>(12) = trans + R * (Xk_.segment<3>(12) - res_.position);
    Xk_.segment<3>(15) = trans + R * (Xk_.segment<3>(15) - res_.position);
    Xk_.segment<3>(18) = trans + R * (Xk_.segment<3>(18) - res_.position);

    // apply transformation matrix R to variance Pk_
    // and get R * Pk_ * R.transpose
    const Eigen::Matrix3d rot_mat = R.toRotationMatrix();
    Pk_.middleRows<3>(3).applyOnTheLeft(rot_mat);

    for (int i = 0; i < 4; i++)
    {
        Pk_.middleRows<3>(9 + i * 3)
                = rot_mat
                * (Pk_.middleRows<3>(9 + i * 3) - Pk_.topRows<3>());
    }

    Pk_.topRows<3>().setZero();
    Pk_.middleRows<3>(6).setZero();

    Pk_.middleCols<3>(3).applyOnTheRight(rot_mat.transpose());

    for (int i = 0; i < 4; i++)
    {
        Pk_.middleCols<3>(9 + i * 3)
                = (Pk_.middleCols<3>(9 + i * 3) - Pk_.leftCols<3>())
                * rot_mat.transpose();
    }

    Pk_.leftCols<3>().setZero();
    Pk_.middleCols<3>(6).setZero();

    res_.position = trans;
    res_.orientation = rot;
}

} /* estimator */

} /* dog_control */
