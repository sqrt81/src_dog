#include "dog_control/control/WholeBodyController.h"

#include "dog_control/control/MPCBase.h"
#include "dog_control/control/TrajectoryController.h"
#include "dog_control/hardware/ClockBase.h"
#include "dog_control/physics/DogModel.h"
#include "dog_control/control/FootPosController.h"
#include "dog_control/optimization/QuadSolver.h"
#include "dog_control/physics/EigenToolbox.h"
#include "dog_control/utils/MiniLog.h"
#include "dog_control/utils/Math.h"

namespace dog_control
{

namespace control
{

namespace
{

constexpr int n_s = 6;         // 6 spatial joints
constexpr int n_j = 12 + n_s;  // 18 joints (12 revolute + 6 spatial)
constexpr int n_f = 3 * 4;     // 3 force params (fx, fy, fz) per foot
constexpr int n_v = n_s + n_f; // revolute joints are excluded
constexpr int n_ci = 4 * 4;    // 4 inequality constraints per foot

inline Eigen::MatrixXd DCInv(const Eigen::MatrixXd &A,
                             const Eigen::MatrixXd &inv_H,
                             bool& invertable)
{
    const Eigen::MatrixXd tmp = inv_H * A.transpose();
    const Eigen::MatrixXd AHA = A * tmp;

    // If singularity occurs, disable this term.
    if (utils::is_zero(AHA.determinant()))
    {
        invertable = false;
        return Eigen::MatrixXd::Zero(A.cols(), A.rows());
    }
    else
    {
        invertable = true;
        return tmp * AHA.inverse();
    }
}

} /* anonymous */

WholeBodyController::WholeBodyController()
{
    mass_.resize(n_j, n_j);
    inv_m_.resize(n_j, n_j);
    force_bias_.resize(n_j);
    vq_.resize(n_j);
    jacobian_.resize(3, n_j);
    null_space_.resize(n_j - n_s, n_j);
    null_space_i_.resize(n_j, n_j);

    G_ = Eigen::MatrixXd::Zero(n_v, n_v);
    g0_.resize(n_v);
    CE_.resize(n_v, n_s);
    ce0_.resize(n_s);
    CI_.resize(n_v, n_ci);
    ci0_ = Eigen::VectorXd::Zero(n_ci);
}

void WholeBodyController::Initialize(utils::ParamDictCRef dict)
{
    kp_body_Cartesian_ = ReadParOrDie(
                dict, PARAM_WITH_NS(kp_body_Cartesian, control/WBC));
    kd_body_Cartesian_ = ReadParOrDie(
                dict, PARAM_WITH_NS(kd_body_Cartesian, control/WBC));
    kp_body_rotation_ = ReadParOrDie(
                dict, PARAM_WITH_NS(kp_body_rotation, control/WBC));
    kd_body_rotation_ = ReadParOrDie(
                dict, PARAM_WITH_NS(kd_body_rotation, control/WBC));
    kp_foot_Cartesian_ = ReadParOrDie(
                dict, PARAM_WITH_NS(kp_foot_Cartesian, control/WBC));
    kd_foot_Cartesian_ = ReadParOrDie(
                dict, PARAM_WITH_NS(kd_foot_Cartesian, control/WBC));

    q_f_ = ReadParOrDie(dict, PARAM_WITH_NS(force_loss, control/WBC));
    q_j_ = ReadParOrDie(dict, PARAM_WITH_NS(acc_loss, control/WBC));
    ground_friction_ = ReadParOrDie(
                dict, PARAM_WITH_NS(ground_friction, physics));

    gravity_ = Eigen::Vector3d(
                ReadParOrDie(dict, PARAM_WITH_NS(gravity_x, physics)),
                ReadParOrDie(dict, PARAM_WITH_NS(gravity_y, physics)),
                ReadParOrDie(dict, PARAM_WITH_NS(gravity_z, physics)));

    for(int i = 0; i < n_s; i++)
        G_(i, i) = q_j_;

    for(int i = n_s; i < n_v; i++)
        G_(i, i) = q_f_;

    for(int i = 0; i < 4; i++)
    {
        int row_offset = n_s + i * 3;
        int col_offset = i * 4;

        CI_(row_offset + 0, col_offset + 0) =            - 1.0;
        CI_(row_offset + 2, col_offset + 0) = ground_friction_;
        CI_(row_offset + 0, col_offset + 1) =              1.0;
        CI_(row_offset + 2, col_offset + 1) = ground_friction_;
        CI_(row_offset + 1, col_offset + 2) =            - 1.0;
        CI_(row_offset + 2, col_offset + 2) = ground_friction_;
        CI_(row_offset + 1, col_offset + 3) =              1.0;
        CI_(row_offset + 2, col_offset + 3) = ground_friction_;
    }
}

void WholeBodyController::ConnectClock(
        boost::shared_ptr<hardware::ClockBase> clock)
{
    clock_ptr_ = clock;
}

void WholeBodyController::ConnectModel(
        boost::shared_ptr<physics::DogModel> model)
{
    model_ptr_ = model;
}

void WholeBodyController::ConnectMPC(
        boost::shared_ptr<control::MPCBase> mpc)
{
    mpc_ptr_ = mpc;
}

void WholeBodyController::ConnectTraj(
        boost::shared_ptr<control::TrajectoryController> traj)
{
    traj_ptr_ = traj;
}

void WholeBodyController::SetPipelineData(
        boost::shared_ptr<message::MotorCommand> cmd)
{
    cmd_ = cmd;
}

void WholeBodyController::SetTorsoMotionTask(
        FBStateCRef state_desired,
        const Eigen::Vector3d &a_lin_desired,
        const Eigen::Vector3d &a_rot_desired)
{
    torso_task_.state = state_desired;
    torso_task_.linear_acc = a_lin_desired;
    torso_task_.rot_acc = a_rot_desired;
}

void WholeBodyController::SetFootMotionTask(FootStateCRef state_desired,
                                            const Eigen::Vector3d &a_desired)
{
    VALID_LEGNAME(state_desired.foot_name);

    foot_state_task_[state_desired.foot_name] = state_desired;
    foot_acc_task_[state_desired.foot_name] = a_desired;
}

void WholeBodyController::SetRefFootForces(
        const std::array<Eigen::Vector3d, 4> &foot_forces,
        const std::array<bool, 4> &foot_contact)
{
    ref_force_ = foot_forces;
    foot_contact_ = foot_contact;
}

void WholeBodyController::Update()
{
    boost::shared_ptr<control::MPCBase> mpc
            = mpc_ptr_.lock();
    CHECK(mpc) << "[WBC] MPC is not set!";

    boost::shared_ptr<hardware::ClockBase> clock = clock_ptr_.lock();
    CHECK(clock) << "[WBC] clock is not set!";

    const double cur_time = clock->Time();

    mpc->GetFeetForce(cur_time, ref_force_, foot_contact_);

    boost::shared_ptr<control::TrajectoryController> traj = traj_ptr_.lock();
    CHECK(traj) << "[WBC] traj is not set!";
    torso_task_ = traj->GetTorsoState(cur_time);

    boost::shared_ptr<physics::DogModel> model = model_ptr_.lock();
    CHECK(model) << "[WBC] Model is not set!";

    const message::FloatingBaseState torso_state = model->TorsoState();
    mass_ = model->MassMatrix();
    force_bias_ = model->BiasForces();
    inv_m_ = mass_.inverse();
    vq_ = model->Vq();
    foot_contact_ = model->FootContact();

    CE_.topRows<n_s>() = mass_.topLeftCorner<n_s, n_s>();

    for (int i = 0; i < 4; i++)
    {
        traj->GetFootState(cur_time, static_cast<message::LegName>(i),
                           foot_state_task_[i].pos,
                           foot_state_task_[i].vel,
                           foot_acc_task_[i],
                           foot_contact_[i]);

        if (!foot_contact_[i])
        {
            Eigen::Vector3d local_pos;
            Eigen::Vector3d local_vel;
            Eigen::Vector3d local_acc;

            traj->GetSwingFootLocalState(
                        cur_time, static_cast<message::LegName>(i),
                        local_pos, local_vel, local_acc);

            foot_state_task_[i].pos
                    = torso_state.trans + torso_state.rot * local_pos;
            foot_state_task_[i].vel
                    = torso_state.rot
                    * (torso_state.linear_vel + local_vel
                       + torso_state.rot_vel.cross(local_pos));
            foot_acc_task_[i] = torso_state.rot * (
                        torso_task_.linear_acc + local_acc
                        - torso_state.rot_vel.squaredNorm() * local_pos
                        + torso_task_.rot_acc.cross(local_pos)
                        + 2 * torso_state.rot_vel.cross(local_vel));
        }
    }

    // command acceleration contains error feedback:
    // a_cmd = a_desired + Kp * delta_x + Kd * delta_v
    Eigen::Vector3d a_cmd;

    // The first is body rotational acceleration task.
    Eigen::Quaterniond rot_pos_err
            = torso_state.rot.conjugate() * torso_task_.state.rot;

    if (rot_pos_err.w() < 0)
        rot_pos_err.coeffs() *= -1;

    a_cmd = torso_state.rot.conjugate() * torso_task_.rot_acc
            + kp_body_rotation_ * physics::QuatToSO3(rot_pos_err)
            + kd_body_rotation_
              * (rot_pos_err * torso_task_.state.rot_vel
                 - torso_state.rot_vel);

    jacobian_.setZero();
    jacobian_.block<3, 3>(0, 0).diagonal().setOnes();

    // aq is the desired joint acceleration, which will be
    // iteratively updated to complete the tasks.
    bool invertable;
    Eigen::VectorXd aq = DCInv(jacobian_, inv_m_, invertable) * a_cmd;

    // The second task is body linear acceleration task.
    // Different from rotational acceleration task,
    // linear acceleration task is computed in global frame.
    // As a result, velocity difference should be transformed
    // into local frame to keep jacobian being simpliy identidy.
    a_cmd = torso_task_.linear_acc
            + kp_body_Cartesian_
              * (torso_task_.state.trans - torso_state.trans)
            + kd_body_Cartesian_
              * (torso_task_.state.rot * torso_task_.state.linear_vel
                 - torso_state.rot * torso_state.linear_vel);

    a_cmd = torso_state.rot.conjugate() * a_cmd;

//    Eigen::MatrixXd null_space
//            = Eigen::MatrixXd::Identity(n_j, n_j)
//            - jacobian.transpose() * jacobian;
    null_space_.setZero();
    null_space_.rightCols<n_j - n_s>().diagonal().setOnes();

    jacobian_.block<3, 3>(0, 0).diagonal().setZero();
    jacobian_.block<3, 3>(0, 3).diagonal().setOnes();

    // Here, null_space is very simple
    // (diagonal with first 3 elements zero).
    // So delta_aq is easily computed.
//    aq += null_space * DCInv(jacobian * null_space, inv_m, invertable)
//            * (a_cmd - jacobian * aq);
    aq.tail<n_j - 3>()
            += (DCInv(jacobian_, inv_m_, invertable)
                * (a_cmd - aq.segment<3>(3))).tail<n_j - 3>();

    Eigen::VectorXd res_opt(n_v);
    std::array<Eigen::MatrixXd, 4> local_jacob;

    bool leg_inv[4]; // whether leg jacobian is invertable

    // The next tasks are foot positions.
    // Generally, the formula for delta_aq_i
    // (increasement of aq caused by task i) is
    // N * (J * N).PsudoInverse() * (a_cmd_i - vJ_i * vq - J_i * aq_(i - 1)).
    // However, for torso tasks, vJ_i * vq = 0, so that term is omitted.
    for (int i = 0; i < 4; i++)
    {
        // null_space_i_ = Identity() - jacobian.transpose() * jacobian
        null_space_i_ = - jacobian_.transpose() * jacobian_;
        null_space_i_.diagonal().array() += 1.;
        null_space_.applyOnTheRight(null_space_i_);
        jacobian_ = model->FullJacob(static_cast<message::LegName>(i));

        a_cmd = foot_acc_task_[i]
                + kp_foot_Cartesian_
                  * (foot_state_task_[i].pos
                     - model->FootPos(static_cast<message::LegName>(i)))
                + kd_foot_Cartesian_
                  * (foot_state_task_[i].vel
                     - model->FootVel(static_cast<message::LegName>(i)));

        // The first 6 elements of aq is decided completely
        // by torso acceleration.
        // So that null space for joint acceleration to operate in
        // must be orthogonal to the first 6 unit vector.
        // The above derivation indicates that the top 6 rows
        // of the null_space matrix are all zero.
        // And we can take advantage of it.
        aq.tail<n_f>() += null_space_
                * DCInv(jacobian_.rightCols<n_f>() * null_space_,
                        inv_m_, leg_inv[i])
                * (a_cmd - model->VJDotVq(static_cast<message::LegName>(i))
                   - jacobian_ * aq);

        if (foot_contact_[i])
        {
            CE_.middleRows<3>(n_s + i * 3) = - jacobian_.leftCols<n_s>();
            local_jacob[i] = jacobian_.rightCols<n_j - n_s>();
            res_opt.segment<3>(n_s + i * 3) = ref_force_[i];
        }
        else // Disable foot force if foot i is not in contact.
        {
            CE_.middleRows<3>(n_s + i * 3).setIdentity();
            res_opt.segment<3>(n_s + i * 3).setZero();
        }
    }

    // compensate gravity
    aq.segment<3>(3) -= torso_state.rot.conjugate() * gravity_;
    res_opt.head<n_s>() = aq.head<n_s>();
    Eigen::VectorXd res0 = res_opt;
    g0_ = - res_opt.cwiseProduct(G_.diagonal());
    ce0_ = force_bias_.head<n_s>()
            + mass_.topRightCorner<n_s, n_j - n_s>() * aq.tail<n_j - n_s>();

    optimization::SolveQuadProg(G_, g0_, CE_, ce0_, CI_, ci0_, res_opt);

    // tau = H * aq + C - J.transpose() * f_foot
    Eigen::VectorXd torq
            = mass_.bottomLeftCorner<n_j - n_s, n_s>() * res_opt.head<n_s>()
            + mass_.bottomRightCorner<n_j - n_s, n_j - n_s>()
              * aq.tail<n_j - n_s>()
            + force_bias_.tail<n_j - n_s>()
            - model->Friction();

    torq.setZero();

    for (int i = 0; i < 4; i++)
    {
        if (foot_contact_[i])
        {
            torq.noalias() -= local_jacob[i].transpose()
                    * res_opt.segment<3>(n_s + i * 3);
//            torq.noalias() -= local_jacob[i].transpose()
//                    * ref_force_[i];
        }
    }

    for (int i = 0; i < 4; i++)
    {
        if (leg_inv[i])
        {
            cmd_->at(i * 3    ).torq = torq(i * 3    );
            cmd_->at(i * 3 + 1).torq = torq(i * 3 + 1);
            cmd_->at(i * 3 + 2).torq = torq(i * 3 + 2);
        }
        else
        {
            cmd_->at(i * 3    ).torq = 0;
            cmd_->at(i * 3 + 1).torq = 0;
            cmd_->at(i * 3 + 2).torq = 0;
        }
    }
}

} /* control */

} /* dog_control */
