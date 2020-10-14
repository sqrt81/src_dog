#include "dog_control/control/ModelPredictiveController.h"

#include "dog_control/control/WholeBodyController.h"
#include "dog_control/hardware/ClockBase.h"
#include "dog_control/optimization/QuadSolver.h"
#include "dog_control/physics/EigenToolbox.h"
#include "dog_control/physics/DogModel.h"
#include "dog_control/utils/MiniLog.h"
#include "dog_control/utils/Math.h"

namespace dog_control
{

namespace control
{

namespace
{

constexpr int n_s = 13;         // number of stats
constexpr int n_f = 12;         // number of forces

} /* anonymous */

ModelPredictiveController::ModelPredictiveController()
{
}

void ModelPredictiveController::Initialize(utils::ParamDictCRef dict)
{
    ground_friction_ = ReadParOrDie(
                dict, PARAM_WITH_NS(ground_friction, physics));
    f_z_max_ = ReadParOrDie(dict, PARAM_WITH_NS(f_z_max, control/MPC));
    f_z_min_ = ReadParOrDie(dict, PARAM_WITH_NS(f_z_min, control/MPC));
    gravity_ = Eigen::Vector3d(
                ReadParOrDie(dict, PARAM_WITH_NS(gravity_x, physics)),
                ReadParOrDie(dict, PARAM_WITH_NS(gravity_y, physics)),
                ReadParOrDie(dict, PARAM_WITH_NS(gravity_z, physics)));
    inv_inertia_ = Eigen::Vector3d(
                ReadParOrDie(dict, PARAM_WITH_NS(I0xx, model)),
                ReadParOrDie(dict, PARAM_WITH_NS(I0yy, model)),
                ReadParOrDie(dict, PARAM_WITH_NS(I0zz, model)))
            .cwiseInverse().asDiagonal();
    inv_mass_ = 1.0 /
            (ReadParOrDie(dict, PARAM_WITH_NS(torso_mass, model))
             + 4 * (ReadParOrDie(dict, PARAM_WITH_NS(hip_mass, model))
                    + ReadParOrDie(dict, PARAM_WITH_NS(thigh_mass, model))
                    + ReadParOrDie(dict, PARAM_WITH_NS(shin_mass, model))));

    pred_horizon_ = utils::round(
                ReadParOrDie(dict, PARAM_WITH_NS(pred_horizon, control/MPC)));
    pred_interval_ = ReadParOrDie(
                dict, PARAM_WITH_NS(pred_interval, control/MPC));
    update_period_ = ReadParOrDie(
                dict, PARAM_WITH_NS(update_period, control/MPC));

    state_weight_.resize(pred_horizon_ * n_s);
    state_weight_.segment<3>(0).setConstant(
                ReadParOrDie(dict, PARAM_WITH_NS(rot_w, control/MPC)));
    state_weight_.segment<3>(3).setConstant(
                ReadParOrDie(dict, PARAM_WITH_NS(pos_w, control/MPC)));
    state_weight_.segment<3>(6).setConstant(
                ReadParOrDie(dict, PARAM_WITH_NS(rot_vel_w, control/MPC)));
    state_weight_.segment<3>(9).setConstant(
                ReadParOrDie(dict, PARAM_WITH_NS(vel_w, control/MPC)));
    state_weight_(n_s - 1) = 0;
    force_weight_ = ReadParOrDie(dict, PARAM_WITH_NS(force_w, control/MPC));

    for(unsigned int i = 1; i < pred_horizon_; i++)
        state_weight_.segment<n_s>(i * n_s) = state_weight_.head<n_s>();

    update_dt_ = ReadParOrDie(dict, PARAM_WITH_NS(control_period, control));
    last_update_time_ = 0.;

    A_.resize(pred_horizon_, Eigen::MatrixXd::Zero(n_s, n_s));
    B_.resize(pred_horizon_, Eigen::MatrixXd::Zero(n_s, n_f));

    Aqp_Bqp_ = Eigen::MatrixXd::Zero(pred_horizon_ * n_s,
                                     pred_horizon_ * n_f);
    Aqp_C_X0_ = Eigen::VectorXd::Zero(pred_horizon_ * n_s);
    pred_force_.resize(pred_horizon_ * n_f);

    foot_force_.resize(pred_horizon_,
                       {Eigen::Vector3d::Zero(),
                        Eigen::Vector3d::Zero(),
                        Eigen::Vector3d::Zero(),
                        Eigen::Vector3d::Zero()});

    G_.resize(pred_horizon_ * n_f, pred_horizon_ * n_f);
    g0_.resize(pred_horizon_ * n_f);
//    CI_.resize(pred_horizon_ * n_f, pred_horizon_ * n_ci);
//    ci0_.resize(pred_horizon_ * n_ci);
//    CE_.resize(pred_horizon_ * n_f, 0);
//    ce0_.resize(0);
}

void ModelPredictiveController::ConnectWBC(
        boost::shared_ptr<control::WholeBodyController> WBC)
{
    WBC_ptr_ = WBC;
}

void ModelPredictiveController::ConnectModel(
        boost::shared_ptr<physics::DogModel> model)
{
    model_ptr_ = model;
}

void ModelPredictiveController::SetDesiredTorsoTrajectory(
        const TorsoTraj &torso_traj)
{
    desired_traj_ = torso_traj;
}

void ModelPredictiveController::SetCurTorsoPose(FBSCRef cur_state)
{
    cur_state_ = cur_state;
}

void ModelPredictiveController::SetFeetPose(
        const FeetPosSeq &feet_pos, const FeetContactSeq &feet_contact)
{
    feet_pos_seq_ = feet_pos;
    feet_contact_seq_ = feet_contact;
}

void ModelPredictiveController::Update()
{
    boost::shared_ptr<WholeBodyController> wbc = WBC_ptr_.lock();
    CHECK(wbc) << "[MPC] wbc is not set!";

    boost::shared_ptr<hardware::ClockBase> clock = clock_ptr_.lock();
    CHECK(clock) << "[MPC] clock is not set!";

    const double interval = clock->Time() - last_update_time_;

    const int cur_index = static_cast<int>(interval / pred_interval_);
    wbc->SetRefFootForces(foot_force_[cur_index],
                          feet_contact_seq_[cur_index]);

    if(interval < update_period_)
        return;

    CHECK(desired_traj_.size() >= static_cast<unsigned>(pred_horizon_) + 1)
            << "[MPC] torso trajectory length " << desired_traj_.size()
            << " is smaller than prediction horizon " << pred_horizon_
            << " + 1";

    CHECK(feet_pos_seq_.size() >= static_cast<unsigned>(pred_horizon_))
            << "[MPC] feet pos sequence length " << feet_pos_seq_.size()
            << " is smaller than prediction horizon " << pred_horizon_;

    CHECK(feet_contact_seq_.size() >= static_cast<unsigned>(pred_horizon_))
            << "[MPC] feet contact sequence length "
            << feet_contact_seq_.size()
            << " is smaller than prediction horizon " << pred_horizon_;

    boost::shared_ptr<physics::DogModel> model = model_ptr_.lock();
    CHECK(model) << "[MPC] model is not set!";
    cur_state_ = model->TorsoState();

    const double t_t_2 = utils::square(pred_interval_) * 0.5;
    const Eigen::Quaterniond rot_base = desired_traj_[0].rot.conjugate();
    const Eigen::Matrix3d rot_base_mat = rot_base.toRotationMatrix();

    int foot_contact_cnt = 0;

    for (unsigned int i = 0; i < pred_horizon_; i++)
    {
        const message::FloatingBaseState& fb = desired_traj_[i];
        const Eigen::Matrix3d rot = fb.rot.toRotationMatrix();
        const Eigen::Matrix3d inv_iner_global
                = rot * inv_inertia_ * rot.transpose();

        Eigen::MatrixXd& Ai = A_[i];
        Eigen::MatrixXd& Bi = B_[i];

        // Note that the state vector Xi has the form
        // [q p v_rot v_lin 1].transpose(),
        // where q is orientation expressed in quaternion.

        // Build x0_[i], be aware to convert velocities into global frame.
        if(i >= 1)
        {
            const int offset = (pred_horizon_ - i) * n_s;
            Aqp_C_X0_.segment<3>(offset    )
                    = physics::QuatToSO3(rot_base * fb.rot);
            Aqp_C_X0_.segment<3>(offset + 3) = fb.trans;
            Aqp_C_X0_.segment<3>(offset + 6) = fb.rot * fb.rot_vel;
            Aqp_C_X0_.segment<3>(offset + 9) = fb.rot * fb.linear_vel;
            Aqp_C_X0_(offset + n_s - 1) = 1.;
        }

        // Build A_[i].
        Ai.diagonal().setOnes();

        // theta[i + 1] = theta[i] + v_rot_rot_base[i] * dt
        Ai.block<3, 3>(0, 6) = pred_interval_ * rot_base_mat;

        // p[i + 1] = p[i] + v_linear[i] * dt
        // v_lin[i + 1] = v_lin[i] + a_foot[i] * dt + gravity_ * dt
        Ai.block<3, 3>(3, 9).diagonal().setConstant(pred_interval_);
        Ai.block<3, 1>(3, 12) = gravity_ * t_t_2;
        Ai.block<3, 1>(9, 12) = gravity_ * pred_interval_;

        // Build B_[i].
        for (int j = 0; j < 4; j++)
        {
            if (!feet_contact_seq_[i][j])
            {
                Bi.middleCols<3>(j * 3).setZero();
                continue;
            }

            foot_contact_cnt++;

            const Eigen::Matrix3d rel_pos
                    = physics::ToLowerMatrix(feet_pos_seq_[i][j] - fb.trans);

            // a_foot[i] = foot_force[i] / m
            Bi.block<3, 3>(9, j * 3).diagonal().setConstant(
                        inv_mass_ * pred_interval_);
            Bi.block<3, 3>(3, j * 3).diagonal().setConstant(
                        inv_mass_ * t_t_2);

            // a_rot[i] = inertia.inverse() * (rel_pos x foot_force[i])
            Bi.block<3, 3>(6, j * 3)
                    = inv_iner_global * rel_pos * pred_interval_;
            Bi.block<3, 3>(0, j * 3)
                    = Ai.block<3, 3>(0, 6) * Bi.block<3, 3>(6, j * 3) * 0.5;
        }
    }

    {
        // While system matrixs Ai, Bi indexes from 0 to pred_horizon_ - 1,
        // x0_i indexes from 1 to pred_horizon_.
        const message::FloatingBaseState& fb = desired_traj_[pred_horizon_];
        Aqp_C_X0_.segment<3>(0) = physics::QuatToSO3(rot_base * fb.rot);
        Aqp_C_X0_.segment<3>(3) = fb.trans;
        Aqp_C_X0_.segment<3>(6) = fb.rot * fb.rot_vel;
        Aqp_C_X0_.segment<3>(9) = fb.rot * fb.linear_vel;
        Aqp_C_X0_(n_s - 1) = 1.;
    }

    Eigen::Matrix<double, n_s, 1> Ai_x_cur;

    {
        // x0_0 is not used. Instead, current real state cur_state_ is used
        // as the initial state of MPC.
        Ai_x_cur.segment<3>(0) = physics::QuatToSO3(rot_base * cur_state_.rot);
        Ai_x_cur.segment<3>(3) = cur_state_.trans;
        Ai_x_cur.segment<3>(6)
                = cur_state_.rot * cur_state_.rot_vel;
        Ai_x_cur.segment<3>(9)
                = cur_state_.rot * cur_state_.linear_vel;
        Ai_x_cur(n_s - 1) = 1.;
    }

    // Aqp.inverse() is, in fact, easy to compute, since Aqp has
    // all blocks zero except for diagonal and sub-diagonal blocks,
    // and all diagonal blocks of Aqp are identity.
    // Specifically, Aqp.inverse().block(i, j) =
    //      A_[N - i - 1] * ... * A_[N - j]     if i < j
    //      Identity                            if i == j
    //      Zero                                if i > j.
    for (unsigned int i = 0; i < pred_horizon_; i++)
    {
        const int col = (pred_horizon_ - i - 1) * n_f;
        int row = (pred_horizon_ - i - 1) * n_s;

        Ai_x_cur.applyOnTheLeft(A_[i]);
        Aqp_C_X0_.segment<n_s>(row) -= Ai_x_cur;

        Aqp_Bqp_.block<n_s, n_f>(row, col) = B_[i];

        for(unsigned int j = i + 1; j < pred_horizon_; j++)
        {
            row -= n_s;

            Aqp_Bqp_.block<n_s, n_f>(row, col)
                    = A_[j] * Aqp_Bqp_.block<n_s, n_f>(row + n_s, col);
        }
    }

//    LOG(INFO) << "X0     " << Aqp_C_X0_.tail<n_s>().transpose();
//    LOG(INFO) << "X_real " << Ai_x_cur.transpose();

    G_ = Aqp_Bqp_.transpose() * state_weight_.asDiagonal() * Aqp_Bqp_;
    G_.diagonal().array() += force_weight_;
//    g0_ = - Aqp_Bqp_.transpose() * state_weight_.asDiagonal() * Aqp_C_X0_;
    g0_ = - Aqp_Bqp_.transpose() * state_weight_.cwiseProduct(Aqp_C_X0_);

    // Build constraints
    CI_ = Eigen::MatrixXd::Zero(pred_horizon_ * n_f, foot_contact_cnt * 6);
    ci0_.resize(foot_contact_cnt * 6);
    CE_ = Eigen::MatrixXd::Zero(pred_horizon_ * n_f,
                                (pred_horizon_ * 4 - foot_contact_cnt) * 3);
    ce0_ = Eigen::VectorXd::Zero((pred_horizon_ * 4 - foot_contact_cnt) * 3);
    int col_offset = 0;
    int col_offset_eq = 0;

    for (unsigned int i = 0; i < pred_horizon_; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            if (feet_contact_seq_[i][j])
            {
                // first type of inequality constraint is z force constraint
                CI_(i * n_f + j * 3 + 2, col_offset    ) =   1.;
                CI_(i * n_f + j * 3 + 2, col_offset + 1) = - 1.;
                ci0_(col_offset    ) = - f_z_min_;
                ci0_(col_offset + 1) =   f_z_max_;

                // second type is maxium friction constraint
                CI_(i * n_f + j * 3    , col_offset + 2) =               1.;
                CI_(i * n_f + j * 3 + 2, col_offset + 2) = ground_friction_;
                CI_(i * n_f + j * 3    , col_offset + 3) =             - 1.;
                CI_(i * n_f + j * 3 + 2, col_offset + 3) = ground_friction_;
                CI_(i * n_f + j * 3 + 1, col_offset + 4) =               1.;
                CI_(i * n_f + j * 3 + 2, col_offset + 4) = ground_friction_;
                CI_(i * n_f + j * 3 + 1, col_offset + 5) =             - 1.;
                CI_(i * n_f + j * 3 + 2, col_offset + 5) = ground_friction_;
                ci0_(col_offset + 2) = 0.;
                ci0_(col_offset + 3) = 0.;
                ci0_(col_offset + 4) = 0.;
                ci0_(col_offset + 5) = 0.;

                col_offset += 6;
            }
            else
            {
                // If foot is not in contact, set zero force constraint.
                CE_(i * n_f + j * 3    , col_offset_eq    ) = 1.;
                CE_(i * n_f + j * 3 + 1, col_offset_eq + 1) = 1.;
                CE_(i * n_f + j * 3 + 2, col_offset_eq + 2) = 1.;

                col_offset_eq += 3;
            }
        }
    }

    optimization::SolveQuadProg(G_, g0_, CE_, ce0_, CI_, ci0_, pred_force_);

    for (unsigned int i = 0; i < pred_horizon_; i++)
    {
        std::array<Eigen::Vector3d, 4>& force_i
                = foot_force_[pred_horizon_ - i - 1];

        force_i[0] = pred_force_.segment<3>(i * n_f    );
        force_i[1] = pred_force_.segment<3>(i * n_f + 3);
        force_i[2] = pred_force_.segment<3>(i * n_f + 6);
        force_i[3] = pred_force_.segment<3>(i * n_f + 9);
    }

//    LOG(INFO) << "A0" << std::endl << A_[0];
//    LOG(INFO) << "B0" << std::endl << B_[0];

//    LOG(INFO) << "fl: " << foot_force_[0][0].transpose();
//    LOG(INFO) << "fr: " << foot_force_[0][1].transpose();
//    LOG(INFO) << "bl: " << foot_force_[0][2].transpose();
//    LOG(INFO) << "br: " << foot_force_[0][3].transpose() << std::endl;

    last_update_time_ = clock->Time();
}

} /* control */

} /* dog_control */
