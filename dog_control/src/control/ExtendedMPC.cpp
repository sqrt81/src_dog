#include "dog_control/control/ExtendedMPC.h"

#include "dog_control/control/TrajectoryController.h"
#include "dog_control/hardware/ClockBase.h"
#include "dog_control/message/FootState.h"
#include "dog_control/optimization/QuadSolver.h"
#include "dog_control/physics/EigenToolbox.h"
#include "dog_control/physics/DogModel.h"
#include "dog_control/physics/SimDogModel.h"
#include "dog_control/utils/MiniLog.h"
#include "dog_control/utils/Math.h"

namespace dog_control
{

namespace control
{

namespace
{

constexpr int n_s = 12;         // number of stats
constexpr int n_f = 12;         // number of forces
constexpr int n_j = 12;         // number of joints

} /* anonymous */

void ExtendedMPC::Initialize(utils::ParamDictCRef dict)
{
    pred_model_.reset(new physics::SimDogModel());
    pred_model_->Initialize(dict);

    ground_friction_ = ReadParOrDie(
                dict, PARAM_WITH_NS(ground_friction, physics));
    f_z_max_ = ReadParOrDie(dict, PARAM_WITH_NS(f_z_max, control/MPC));
    f_z_min_ = ReadParOrDie(dict, PARAM_WITH_NS(f_z_min, control/MPC));
    gravity_ = Eigen::Vector3d(
                ReadParOrDie(dict, PARAM_WITH_NS(gravity_x, physics)),
                ReadParOrDie(dict, PARAM_WITH_NS(gravity_y, physics)),
                ReadParOrDie(dict, PARAM_WITH_NS(gravity_z, physics)));

//    pred_horizon_ = utils::round(
//                ReadParOrDie(dict, PARAM_WITH_NS(pred_horizon,
//                                                 control/MPC)));
//    pred_interval_ = ReadParOrDie(
//                dict, PARAM_WITH_NS(pred_interval, control/MPC));
    pred_interval_seq_.clear();

    for (int i = 0; ; i++)
    {
        const std::string pred_dt_name
                = PARAM_WITH_NS(pred_interval_, control/ExtendedMPC)
                + std::to_string(i + 1);

        const auto iter = dict.find(pred_dt_name);

        if (iter == dict.end())
            break;
        else
            pred_interval_seq_.push_back(iter->second);
    }

    const unsigned int pred_horizon = pred_interval_seq_.size();

    update_period_ = ReadParOrDie(
                dict, PARAM_WITH_NS(update_period, control/MPC));

    state_weight_.resize(pred_horizon * n_s);
    state_weight_.segment<3>(0).setConstant(
                ReadParOrDie(dict, PARAM_WITH_NS(rot_w, control/MPC)));
    state_weight_.segment<3>(3).setConstant(
                ReadParOrDie(dict, PARAM_WITH_NS(pos_w, control/MPC)));
    state_weight_.segment<3>(6).setConstant(
                ReadParOrDie(dict, PARAM_WITH_NS(rot_vel_w, control/MPC)));
    state_weight_.segment<3>(9).setConstant(
                ReadParOrDie(dict, PARAM_WITH_NS(vel_w, control/MPC)));
    force_weight_ = ReadParOrDie(dict, PARAM_WITH_NS(force_w, control/MPC));

    for(unsigned int i = 1; i < pred_horizon; i++)
        state_weight_.segment<n_s>(i * n_s) = state_weight_.head<n_s>();

    update_dt_ = ReadParOrDie(dict, PARAM_WITH_NS(control_period, control));
    last_update_time_ = - 1.;

    A_.resize(pred_horizon, Eigen::MatrixXd::Zero(n_s, n_s));
    B_.resize(pred_horizon, Eigen::MatrixXd::Zero(n_s, n_f));
    C_.resize(pred_horizon, Eigen::VectorXd::Zero(n_s));
    Jvel_.resize(pred_horizon + 1, Eigen::VectorXd::Zero(n_j));

    Aqp_Bqp_ = Eigen::MatrixXd::Zero(pred_horizon * n_s,
                                     pred_horizon * n_f);
    Aqp_Cqp_ = Eigen::VectorXd::Zero(pred_horizon * n_s);
    pred_force_.resize(pred_horizon * n_f);

    foot_force_.resize(pred_horizon,
                       {Eigen::Vector3d::Zero(),
                        Eigen::Vector3d::Zero(),
                        Eigen::Vector3d::Zero(),
                        Eigen::Vector3d::Zero()});

    G_.resize(pred_horizon * n_f, pred_horizon * n_f);
    g0_.resize(pred_horizon * n_f);
}

void ExtendedMPC::Update()
{
    boost::shared_ptr<hardware::ClockBase> clock = clock_ptr_.lock();
    CHECK(clock) << "[MPC] clock is not set!";

    const double cur_time = clock->Time();

    if(cur_time - last_update_time_ < update_period_)
        return;

    boost::shared_ptr<TrajectoryController> traj = traj_ptr_.lock();
    CHECK(traj) << "[MPC] traj controller is not set!";

    traj->SampleTrajFromNow(pred_interval_seq_, desired_traj_);
    traj->SampleFootStateFromNow(pred_interval_seq_,
                                 feet_pos_seq_, contact_seq_);

    const unsigned int pred_horizon = pred_interval_seq_.size();

    CHECK(desired_traj_.size() >= static_cast<unsigned>(pred_horizon) + 1)
            << "[MPC] torso trajectory length " << desired_traj_.size()
            << " is smaller than prediction horizon " << pred_horizon
            << " + 1";

    CHECK(feet_pos_seq_.size() >= static_cast<unsigned>(pred_horizon))
            << "[MPC] feet pos sequence length " << feet_pos_seq_.size()
            << " is smaller than prediction horizon " << pred_horizon;

    CHECK(contact_seq_.size() >= static_cast<unsigned>(pred_horizon))
            << "[MPC] feet contact sequence length "
            << contact_seq_.size()
            << " is smaller than prediction horizon " << pred_horizon;

    boost::shared_ptr<physics::DogModel> model = model_ptr_.lock();
    CHECK(model) << "[MPC] model is not set!";

    // jacob_force is transpose
    // of foot position's jacobian wrt body spatial motion.
    Eigen::Matrix<double, 6, 3> jacob_force[4];
    // trans is a temporary matrix recording force's effection
    // on pose and velocity
    Eigen::Matrix<double, n_s, 6> trans;

    int foot_contact_cnt = 0;

    cur_state_ = model->TorsoState();
    Eigen::VectorXd cur_q = model->Jpos();
    Eigen::VectorXd cur_vq = model->Vq().tail<n_j>();

    message::LegConfiguration conf[4];
    message::FloatingBaseJointState s;
    s.q.resize(n_j);
    s.dq.resize(n_j);

    for (int i = 0; i < 4; i++)
    {
        conf[i].foot_name = static_cast<message::LegName>(i);
        model->GetLegConfig(cur_q.segment<3>(i * 3),
                            conf[i].knee_outwards,
                            conf[i].hip_outwards);
    }

    double sample_time = cur_time;

    // get joint state at the sample points
    for (unsigned int i = 0; i <= pred_horizon; i++)
    {
        Eigen::VectorXd& sample_jvel = Jvel_[i];

        for (int j = 0; j < 4; j++)
        {
            Eigen::Vector3d ft_pos;
            Eigen::Vector3d ft_vel;
            Eigen::Vector3d jpos;
            traj->GetLocalFootState(
                        sample_time, static_cast<message::LegName>(j),
                        ft_pos, ft_vel,
                        conf[j].hip_outwards, conf[j].knee_outwards);

            jpos = model->InverseKinematics(
                        static_cast<message::LegName>(j), ft_pos,
                        conf[j].knee_outwards, conf[j].hip_outwards);
            Eigen::Matrix3d jacob = model->ComputeJacobian(
                        static_cast<message::LegName>(j), jpos);
            bool invertable;
            Eigen::Matrix3d inv_jacob;
            jacob.computeInverseWithCheck(
                        inv_jacob, invertable, utils::precision);

            if (invertable)
            {
                sample_jvel.segment<3>(j * 3) = inv_jacob * ft_vel;
            }
            else
            {
                if (i == 0)
                    sample_jvel.segment<3>(j * 3)
                            = cur_vq.segment<3>(j * 3);
                else
                    sample_jvel.segment<3>(j * 3)
                            = Jvel_[i - 1].segment<3>(j * 3);
            }
        }

        sample_time += pred_interval_seq_[i];
    }

    sample_time = cur_time;

    for (unsigned int i = 0; i < pred_horizon; i++)
    {
        const double pred_interval = pred_interval_seq_[i];
        const double t_t_2 = utils::square(pred_interval) * 0.5;
        const double middle_time = sample_time + 0.5 * pred_interval;
        sample_time += pred_interval;

        FBSCRef this_state = desired_traj_[i];
        FBSCRef next_state = desired_traj_[i + 1];
        Eigen::Matrix<double, n_j, 1> middle_jpos;
        Eigen::Matrix<double, n_j, 1> middle_jvel;
        Eigen::Matrix<double, 18, 1> desired_acc;

        // get desired state between step i and i + 1
        s.base = traj->GetTorsoState(middle_time).state;
        const Eigen::Matrix3d rot_interval
                = s.base.rot.conjugate().toRotationMatrix();

        for (int j = 0; j < 4; j++)
        {
            Eigen::Vector3d ft_pos;
            Eigen::Vector3d ft_vel;
            traj->GetLocalFootState(
                        middle_time, static_cast<message::LegName>(j),
                        ft_pos, ft_vel,
                        conf[j].hip_outwards, conf[j].knee_outwards);

            jacob_force[j].topRows<3>()
                    = physics::ToLowerMatrix(ft_pos) * rot_interval;
            jacob_force[j].bottomRows<3>() = rot_interval;

            middle_jpos.segment<3>(j * 3) = model->InverseKinematics(
                        static_cast<message::LegName>(j), ft_pos,
                        conf[j].knee_outwards, conf[j].hip_outwards);
            Eigen::Matrix3d jacob = model->ComputeJacobian(
                        static_cast<message::LegName>(j),
                        middle_jpos.segment<3>(j * 3));
            bool invertable;
            Eigen::Matrix3d inv_jacob;
            jacob.computeInverseWithCheck(
                        inv_jacob, invertable, utils::precision);

            if (invertable)
            {
                middle_jvel.segment<3>(j * 3) = inv_jacob * ft_vel;
            }
            else
            {
                middle_jvel.segment<3>(j * 3)
                        = (Jvel_[i].segment<3>(j * 3)
                           + Jvel_[i + 1].segment<3>(j * 3)) / 2;
            }
        }

        for (int j = 0; j < n_j; j++)
        {
            s.q[j] = middle_jpos(j);
            s.dq[j] = middle_jvel(j);
        }

        desired_acc.segment<3>(0)
                = next_state.rot_vel - this_state.rot_vel;
        desired_acc.segment<3>(3)
                = next_state.linear_vel - this_state.linear_vel;
        desired_acc.tail<n_j>() = Jvel_[i + 1] - Jvel_[i];

        desired_acc /= pred_interval;
        desired_acc.segment<3>(3) -= rot_interval * gravity_;

        pred_model_->SetJointMotionState(s);
        const physics::spatial::SMat inv_H
                = pred_model_->MassMatrix().topLeftCorner<6, 6>().inverse();

        // force_desired is the total force needed in desired trajectory.
        const physics::spatial::SVec force_desired
                = pred_model_->MassMatrix().topRows<6>() * desired_acc
                + pred_model_->BiasForces().head<6>();
        // base_jacob is base bias force's jacobian wrt velocity.
        const physics::spatial::SMat base_jacob
                = pred_model_->BaseForceJacobian();
        // ext_force_jacob is desired force's jacobian wrt torso pose.
        physics::spatial::SMat ext_force_jacob;
        ext_force_jacob.topLeftCorner<3, 3>()
                = - physics::ToLowerMatrix(force_desired.head<3>());
        ext_force_jacob.bottomLeftCorner<3, 3>()
                = - physics::ToLowerMatrix(force_desired.tail<3>());
        ext_force_jacob.topRightCorner<3, 3>().noalias()
                = ext_force_jacob.bottomLeftCorner<3, 3>() * rot_interval;
        ext_force_jacob.bottomRightCorner<3, 3>().setZero();

        // Build system matrix A_[i], B_[i]
        const Eigen::Matrix3d rot = this_state.rot.toRotationMatrix();
        const Eigen::Matrix3d rel_rot
                = (next_state.rot.conjugate() * this_state.rot)
                  .toRotationMatrix();

        Eigen::MatrixXd& Ai = A_[i];
        Eigen::MatrixXd& Bi = B_[i];
        Eigen::VectorXd& Ci = C_[i];

        const Eigen::Vector3d delta_pos
                = (this_state.rot * this_state.linear_vel
                   + next_state.rot * next_state.linear_vel)
                * (pred_interval * 0.5);

        Ai.setZero();
        Ai.block<3, 3>(0, 0) = rel_rot;
        Ai.block<3, 3>(0, 6) = rel_rot * pred_interval;
        Ai.block<3, 3>(3, 0) = - physics::ToLowerMatrix(delta_pos) * rot;
        Ai.block<3, 3>(3, 3).setIdentity();
        Ai.block<3, 3>(3, 9) = rot * pred_interval;
//        Ai.block<3, 3>(6, 6).setIdentity();
//        Ai.block<3, 3>(9, 9).setIdentity();
        Ai.diagonal().tail<6>().setOnes();

        trans.setZero();
        trans.block<3, 3>(0, 0) = rel_rot * t_t_2;
        trans.block<3, 3>(3, 3) = rot * t_t_2;
        trans.block<6, 6>(6, 0).diagonal().setConstant(pred_interval);
        trans.applyOnTheRight(inv_H);

        for (int j = 0; j < 4; j++)
        {
            if (contact_seq_[i][j])
            {
                Bi.middleCols<3>(j * 3).noalias() = trans * jacob_force[j];
                foot_contact_cnt++;
            }
            else
                Bi.middleCols<3>(j * 3).setZero();
        }

        Ci.noalias() = - trans * force_desired;
        Ci.segment<3>(0) += rel_rot
                * (this_state.rot_vel + next_state.rot_vel)
                * (pred_interval * 0.5);
        Ci.segment<3>(3) += this_state.trans - next_state.trans + delta_pos;

        Eigen::Matrix3d local_g
                = physics::ToLowerMatrix(- rot.transpose() * gravity_);
        Ai.block<3, 3>(3, 0) -= rot * local_g * t_t_2;
        Ai.block<3, 3>(9, 0) -= local_g * pred_interval;
        Ai.leftCols<6>() -= trans * ext_force_jacob;
        Ai.rightCols<6>() -= trans * base_jacob;
    }

    {
        // add difference of X0 into C_[0]
        FBSCRef des_x0 = desired_traj_[0];
        const Eigen::Quaterniond rot_diff
                = des_x0.rot.conjugate() * cur_state_.rot;
        Eigen::Matrix<double, n_s, 1> X0;
        X0.segment<3>(0) = physics::QuatToSO3(rot_diff);
        X0.segment<3>(3) = cur_state_.trans - des_x0.trans;
        X0.segment<3>(6) = cur_state_.rot_vel - des_x0.rot_vel;
        X0.segment<3>(9) = cur_state_.linear_vel - des_x0.linear_vel;
        C_[0] += A_[0] * X0;
    }

    // Aqp.inverse() is, in fact, easy to compute, since Aqp has
    // all blocks zero except for diagonal and sub-diagonal blocks,
    // and all diagonal blocks of Aqp are identity.
    // Specifically, Aqp.inverse().block(i, j) =
    //      A_[N - i - 1] * ... * A_[N - j]     if i < j
    //      Identity                            if i == j
    //      Zero                                if i > j.
    for (unsigned int i = 0; i < pred_horizon; i++)
    {
        const int index = pred_horizon - i - 1;
        const int col = index * n_f;
        int row = index * n_s;

        if (i > 0)
            Aqp_Cqp_.segment<n_s>(row)
                    = A_[i] * Aqp_Cqp_.segment<n_s>(row + n_s) + C_[i];
        else
            Aqp_Cqp_.segment<n_s>(row) = C_[0];

        Aqp_Bqp_.block<n_s, n_f>(row, col) = B_[i];

        for (unsigned int j = i + 1; j < pred_horizon; j++)
        {
            row -= n_s;

            Aqp_Bqp_.block<n_s, n_f>(row, col)
                    = A_[j] * Aqp_Bqp_.block<n_s, n_f>(row + n_s, col);
        }
    }

    G_ = Aqp_Bqp_.transpose() * state_weight_.asDiagonal() * Aqp_Bqp_;
    G_.diagonal().array() += force_weight_;
    g0_ = Aqp_Bqp_.transpose() * state_weight_.cwiseProduct(Aqp_Cqp_);

    // Build constraints
    Eigen::MatrixXd CI = Eigen::MatrixXd::Zero(pred_horizon * n_f,
                                               foot_contact_cnt * 6);
    Eigen::VectorXd ci0(foot_contact_cnt * 6);
    Eigen::MatrixXd CE = Eigen::MatrixXd::Zero(
                pred_horizon * n_f,
                (pred_horizon * 4 - foot_contact_cnt) * 3);
    Eigen::VectorXd ce0 = Eigen::VectorXd::Zero(
                (pred_horizon * 4 - foot_contact_cnt) * 3);
    int col_offset = 0;
    int col_offset_eq = 0;

    for (unsigned int i = 0; i < pred_horizon; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            if (contact_seq_[i][j])
            {
                // first type of inequality constraint is z force constraint
                CI(i * n_f + j * 3 + 2, col_offset    ) =   1.;
                CI(i * n_f + j * 3 + 2, col_offset + 1) = - 1.;
                ci0(col_offset    ) = - f_z_min_;
                ci0(col_offset + 1) =   f_z_max_;

                // second type is maxium friction constraint
                CI(i * n_f + j * 3    , col_offset + 2) =               1.;
                CI(i * n_f + j * 3 + 2, col_offset + 2) = ground_friction_;
                CI(i * n_f + j * 3    , col_offset + 3) =             - 1.;
                CI(i * n_f + j * 3 + 2, col_offset + 3) = ground_friction_;
                CI(i * n_f + j * 3 + 1, col_offset + 4) =               1.;
                CI(i * n_f + j * 3 + 2, col_offset + 4) = ground_friction_;
                CI(i * n_f + j * 3 + 1, col_offset + 5) =             - 1.;
                CI(i * n_f + j * 3 + 2, col_offset + 5) = ground_friction_;
                ci0(col_offset + 2) = 0.;
                ci0(col_offset + 3) = 0.;
                ci0(col_offset + 4) = 0.;
                ci0(col_offset + 5) = 0.;

                col_offset += 6;
            }
            else
            {
                // If foot is not in contact, set zero force constraint.
                CE(i * n_f + j * 3    , col_offset_eq    ) = 1.;
                CE(i * n_f + j * 3 + 1, col_offset_eq + 1) = 1.;
                CE(i * n_f + j * 3 + 2, col_offset_eq + 2) = 1.;

                col_offset_eq += 3;
            }
        }
    }

    optimization::SolveQuadProg(G_, g0_, CE, ce0, CI, ci0, pred_force_);

    for (unsigned int i = 0; i < pred_horizon; i++)
    {
        std::array<Eigen::Vector3d, 4>& force_i
                = foot_force_[pred_horizon - i - 1];

        force_i[0] = pred_force_.segment<3>(i * n_f    );
        force_i[1] = pred_force_.segment<3>(i * n_f + 3);
        force_i[2] = pred_force_.segment<3>(i * n_f + 6);
        force_i[3] = pred_force_.segment<3>(i * n_f + 9);
    }

    last_update_time_ = cur_time;
}

void ExtendedMPC::GetFeetForce(
        double t,
        std::array<Eigen::Vector3d, 4> &force,
        std::array<bool, 4> &contact) const
{
    double interval = std::max(t - last_update_time_, 0.);

    for (unsigned int i = 0; i < pred_interval_seq_.size(); i++)
    {
        interval -= pred_interval_seq_[i];

        if (interval < 0)
        {
            force = foot_force_[i];
            contact = contact_seq_[i];

            return;
        }
    }

    force = foot_force_.back();
    contact = contact_seq_.back();
}

} /* control */

} /* dog_control */

