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

} /* anonymous */

WholeBodyController::WholeBodyController()
{
    mass_.resize(n_j, n_j);
    force_bias_.resize(n_j);
    jacobian_.resize(3, n_j);

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
    CHECK(VALID_LEGNAME(state_desired.foot_name));

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
//    torso_task_ = traj->GetTorsoState(cur_time);
    torso_task_ = traj->GetTorsoState(cur_time);

    boost::shared_ptr<physics::DogModel> model = model_ptr_.lock();
    CHECK(model) << "[WBC] Model is not set!";

    const message::FloatingBaseState torso_state = model->TorsoState();
    mass_ = model->MassMatrix();
    force_bias_ = model->BiasForces();
    Eigen::Matrix<double, n_j - n_s, 1> jpos = model->Jpos();
    Eigen::Matrix<double, n_j - n_s, 1> jvel = model->Vq().tail<n_j - n_s>();

    CE_.topRows<n_s>() = mass_.topLeftCorner<n_s, n_s>();

    for (int i = 0; i < 4; i++)
    {
        bool contact_holder;
        traj->GetCurFootState(static_cast<message::LegName>(i),
                              foot_state_task_[i].pos,
                              foot_state_task_[i].vel,
                              foot_acc_task_[i],
                              contact_holder/*foot_contact_[i]*/);

        if (!foot_contact_[i])
        {
            // If foot is swinging, use local state as task.
            traj->GetSwingFootLocalState(
                        cur_time, static_cast<message::LegName>(i),
                        foot_state_task_[i].pos,
                        foot_state_task_[i].vel,
                        foot_acc_task_[i]);
        }
    }

    // aq is the desired joint acceleration, which will be
    // iteratively updated to complete the tasks.
    Eigen::Matrix<double, n_j, 1> aq;
    // aq0 is joint acceleration without feedback, which is
    // supposed to be smoother than aq.
    Eigen::Matrix<double, n_j - n_s, 1> aq0;

    // command acceleration contains error feedback:
    // a_cmd = a_desired + Kp * delta_x + Kd * delta_v
    Eigen::Vector3d a_cmd;

    // The first is body rotational acceleration task.
    Eigen::Quaterniond rot_pos_err
            = torso_state.rot.conjugate() * torso_task_.state.rot;

    a_cmd = torso_state.rot.conjugate() * torso_task_.rot_acc
            + kp_body_rotation_ * physics::QuatToSO3(rot_pos_err)
            + kd_body_rotation_
              * (rot_pos_err * torso_task_.state.rot_vel
                 - torso_state.rot_vel);
    aq.segment<3>(0) = a_cmd;

    // The second task is body linear acceleration task.
    // Different from rotational acceleration task,
    // linear acceleration task is computed in global frame.
    // As a result, velocity difference should be transformed
    // into local frame to keep jacobian being simpliy identidy.
    a_cmd = kp_body_Cartesian_
            * (torso_task_.state.trans - torso_state.trans)
            + kd_body_Cartesian_
              * (torso_task_.state.rot * torso_task_.state.linear_vel
                 - torso_state.rot * torso_state.linear_vel);

    a_cmd = torso_state.rot.conjugate() * a_cmd + torso_task_.linear_acc;

    aq.segment<3>(3) = a_cmd;

    Eigen::VectorXd res_opt(n_v);
    std::array<Eigen::Matrix3d, 4> foot_jacob;

    for (int i = 0; i < 4; i++)
    {
        const message::LegName leg = static_cast<message::LegName>(i);
        jacobian_ = model->FullJacob(leg);

        bool leg_inv;
        Eigen::Matrix3d inv_j;
        Eigen::Vector3d a_cmd0;

        if (foot_contact_[i])
        {
            a_cmd = foot_acc_task_[i];
//                    + kp_foot_Cartesian_
//                    * (foot_state_task_[i].pos - model->FootPos(leg))
//                    + kd_foot_Cartesian_
//                    * (foot_state_task_[i].vel - model->FootVel(leg));

            // a = J * aq + VJ * vq
            // = J_leg * aq_leg + VJ * vq + J_base * aq_base
            const Eigen::Vector3d a_add = model->VJDotVq(leg)
                    + jacobian_.leftCols<n_s>() * aq.head<n_s>();
            a_cmd -= a_add;
            a_cmd0 = foot_acc_task_[i] - a_add;
            jacobian_.block<3, 3>(0, n_s + i * 3)
                    .computeInverseWithCheck(inv_j, leg_inv,
                                             utils::precision);

            if (!leg_inv)
                inv_j = physics::GeneralInverse(
                            jacobian_.block<3, 3>(0, n_s + i * 3));
        }
        else
        {
            const Eigen::Matrix3d local_jacob = model->LocalJacob(leg);
            a_cmd = foot_acc_task_[i]
                    + kp_foot_Cartesian_
                    * (foot_state_task_[i].pos
                       - model->ComputeLocalPos(leg, jpos.segment<3>(i * 3)));
                    + kd_foot_Cartesian_
                    * (foot_state_task_[i].vel
                       - local_jacob * jvel.segment<3>(i * 3));
            const Eigen::Vector3d a_add
                    = torso_state.rot.conjugate() * model->VJDotVq(leg);
            a_cmd -= a_add;
            a_cmd0 = foot_acc_task_[i] - a_add;

            local_jacob.computeInverseWithCheck(inv_j, leg_inv,
                                                utils::precision);

            if (!leg_inv)
                inv_j = physics::GeneralInverse(local_jacob);
        }

        aq.segment<3>(n_s + i * 3) = inv_j * a_cmd;
        aq0.segment<3>(i * 3) = inv_j * a_cmd0;

        if (foot_contact_[i])
        {
            CE_.middleRows<3>(n_s + i * 3) = - jacobian_.leftCols<n_s>();
            foot_jacob[i] = jacobian_.middleCols<3>(n_s + i * 3);
            res_opt.segment<3>(n_s + i * 3) = ref_force_[i];
        }
        else // Disable foot force if foot i is not in contact.
        {
            CE_.middleRows<3>(n_s + i * 3).setZero();
            res_opt.segment<3>(n_s + i * 3).setZero();
        }
    }

    // compensate gravity
    aq.segment<3>(3) -= torso_state.rot.conjugate() * gravity_;
    res_opt.head<n_s>() = aq.head<n_s>();
    g0_ = - res_opt.cwiseProduct(G_.diagonal());
    ce0_ = force_bias_.head<n_s>()
            + mass_.topRightCorner<n_s, n_j - n_s>() * aq0;

    optimization::SolveQuadProg(G_, g0_, CE_, ce0_, CI_, ci0_, res_opt);

    // tau = H * aq + C - J.transpose() * f_foot
    Eigen::VectorXd torq
            = mass_.bottomLeftCorner<n_j - n_s, n_s>() * res_opt.head<n_s>()
            + mass_.bottomRightCorner<n_j - n_s, n_j - n_s>()
              * aq.tail<n_j - n_s>()
            + force_bias_.tail<n_j - n_s>()
            - model->Friction();

//    LOG(DEBUG) << "aq: " << res_opt.head<n_s>().transpose();
//    LOG(DEBUG) << "ref fl: " << ref_force_[0].transpose();
//    LOG(DEBUG) << "ref fr: " << ref_force_[1].transpose();
//    LOG(DEBUG) << "ref bl: " << ref_force_[2].transpose();
//    LOG(DEBUG) << "ref br: " << ref_force_[3].transpose();
//    LOG(DEBUG) << "act fl: " << res_opt.segment<3>(n_s).transpose();
//    LOG(DEBUG) << "act fr: " << res_opt.segment<3>(n_s + 3).transpose();
//    LOG(DEBUG) << "act bl: " << res_opt.segment<3>(n_s + 6).transpose();
//    LOG(DEBUG) << "act br: " << res_opt.segment<3>(n_s + 9).transpose();

//    torq.setZero();

    for (int i = 0; i < 4; i++)
    {
        if (foot_contact_[i])
        {
            torq.segment<3>(i * 3).noalias()
                    -= foot_jacob[i].transpose()
                    * res_opt.segment<3>(n_s + i * 3);
//            torq.segment<3>(i * 3).noalias()
//                    -= foot_jacob[i].transpose()
//                    * ref_force_[i];
        }
    }

    for (int i = 0; i < n_j - n_s; i++)
    {
        cmd_->at(i).torq = torq(i);
    }
}

} /* control */

} /* dog_control */
