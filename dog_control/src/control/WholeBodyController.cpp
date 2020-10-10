#include "dog_control/control/WholeBodyController.h"
#include "dog_control/optimization/QuadSolver.h"
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

constexpr float singularity = 1e-3;

inline Eigen::MatrixXd DCInv(const Eigen::MatrixXd &A,
                             const Eigen::MatrixXd &inv_H,
                             bool& invertable)
{
    const Eigen::MatrixXd tmp = inv_H * A.transpose();
    const Eigen::MatrixXd AHA = A * tmp;

    // If singularity occurs, disable this term.
    if (utils::abs(AHA.determinant()) < singularity)
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

inline Eigen::Vector3d QuatToSO3(const Eigen::Quaterniond &quat)
{
    Eigen::Vector3d so3;
    so3.x() = quat.x();
    so3.y() = quat.y();
    so3.z() = quat.z();

    const double norm = so3.norm();
    const double theta = 2.0 * asin(norm);

    if (abs(theta) > 1e-6)
        so3 *= theta / norm;
    else
        so3.setZero();

    return so3;
}

inline double ReadParOrDie(utils::ParamDictCRef dict,
                           const std::string &param)
{
    const utils::ParamDict::const_iterator iter = dict.find(param);

    CHECK(iter != dict.end()) << "Param \"" << param << "\" not found.";

    return iter->second;
}

} /* anonymous */

WholeBodyController::WholeBodyController()
{
    G_ = Eigen::MatrixXf::Zero(n_v, n_v);
    g0_.resize(n_v);
    CE_.resize(n_v, n_s);
    ce0_.resize(n_s);
    CI_.resize(n_v, n_ci);
    ci0_ = Eigen::VectorXf::Zero(n_ci);
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
    friction_ = ReadParOrDie(
                dict, PARAM_WITH_NS(ground_friction, physics));

    for(int i = 0; i < n_s; i++)
        G_(i, i) = q_j_;

    for(int i = n_s; i < n_v; i++)
        G_(i, i) = q_f_;

    for(int i = 0; i < 4; i++)
    {
        int row_offset = n_s + i * 3;
        int col_offset = i * 4;

        CI_(row_offset + 0, col_offset + 0) =     - 1.0;
        CI_(row_offset + 2, col_offset + 0) = friction_;
        CI_(row_offset + 0, col_offset + 1) =       1.0;
        CI_(row_offset + 2, col_offset + 1) = friction_;
        CI_(row_offset + 1, col_offset + 2) =     - 1.0;
        CI_(row_offset + 2, col_offset + 2) = friction_;
        CI_(row_offset + 1, col_offset + 3) =       1.0;
        CI_(row_offset + 2, col_offset + 3) = friction_;
    }
}

void WholeBodyController::ConnectModel(
        boost::shared_ptr<physics::DogModel> model)
{
    model_ptr_ = model;
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
    torso_state_task_ = state_desired;
    torso_acc_linear_task_ = a_lin_desired;
    torso_acc_rot_task_ = a_rot_desired;
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
//    static bool t = false;
    boost::shared_ptr<physics::DogModel> model = model_ptr_.lock();
    assert(model);

    message::FloatingBaseState torso_state = model->TorsoState();
    const Eigen::MatrixXd mass = model->MassMatrix();
    const Eigen::VectorXd bias = model->BiasForces();
    const Eigen::MatrixXd inv_m = mass.inverse();
    const Eigen::VectorXd vq = model->Vq();
    const Eigen::Vector3d gravity_(0, 0, 9.8);

    CE_.topRows<n_s>() = mass.topLeftCorner<n_s, n_s>().cast<float>();

    // A typical WBC should compute desired joints' position, velocity
    // and effort according to the given tasks.
    // However, the first two works are finished by FootPosController,
    // so here the WBC only needs to compute effort.

    // command acceleration contains error feedback:
    // a_cmd = a_desired + Kp * delta_x + Kd * delta_v

    Eigen::Vector3d a_cmd;

    // The first is body rotational acceleration task.
    Eigen::Quaterniond rot_pos_err
            = torso_state.rot.conjugate() * torso_state_task_.rot;

    if (rot_pos_err.w() < 0)
        rot_pos_err.coeffs() *= -1;

    // Different from linear acceleration task,
    // rotational acceleration task is computed in torso frame.
    // As a result, velocity difference should be transformed
    // into local frame, while jacobian is simpliy identidy.
    a_cmd = torso_acc_rot_task_
            + kp_body_rotation_ * QuatToSO3(rot_pos_err)
            + kd_body_rotation_
              * (rot_pos_err * torso_state_task_.rot_vel
                 - torso_state.rot_vel);

    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(3, n_j);
    jacobian.block<3, 3>(0, 0).diagonal().setOnes();

    // aq is the desired joint acceleration, which will be
    // iteratively updated to complete the tasks.
    bool invertable;
    Eigen::VectorXd aq = DCInv(jacobian, inv_m, invertable) * a_cmd;
//    LOG(INFO) << "round 0, daq = " << aq.transpose();

    // The second task is body linear acceleration task.
    a_cmd = torso_acc_linear_task_
            + kp_body_Cartesian_
              * (torso_state_task_.trans - torso_state.trans)
            + kd_body_Cartesian_
              * (torso_state_task_.rot * torso_state_task_.linear_vel
                 - torso_state.rot * torso_state.linear_vel);
    a_cmd = torso_state.rot.conjugate() * a_cmd;

//    Eigen::MatrixXd null_space
//            = Eigen::MatrixXd::Identity(n_j, n_j)
//            - jacobian.transpose() * jacobian;
    Eigen::MatrixXd null_space = Eigen::MatrixXd::Zero(n_j - n_s, n_j);
    null_space.rightCols<n_j - n_s>().diagonal().setOnes();

    jacobian.block<3, 3>(0, 0).diagonal().setZero();
    jacobian.block<3, 3>(0, 3).diagonal().setOnes();

    // Here, null_space is very simple
    // (diagonal with first 3 elements zero).
    // So delta_aq is easily computed.
//    aq += null_space * DCInv(jacobian * null_space, inv_m, invertable)
//            * (a_cmd - jacobian * aq);
    aq.tail<n_j - 3>()
            += (DCInv(jacobian, inv_m, invertable)
                * (a_cmd - aq.segment<3>(3))).tail<n_j - 3>();

    Eigen::VectorXf res_opt(n_v);
    std::array<Eigen::MatrixXd, 4> local_jacob;

    bool leg_inv[4]; // whether leg jacobian is invertable

    // The next tasks are foot positions.
    // Generally, the formula for delta_aq_i
    // (increasement of aq caused by task i) is
    // N * (J * N).PsudoInverse() * (a_cmd_i - vJ_i * vq - J_i * aq_(i - 1)).
    // However, for torso tasks, vJ_i * vq = 0, so that term is omitted.
    for (int i = 0; i < 4; i++)
    {
        a_cmd = foot_acc_task_[i]
                + kp_foot_Cartesian_
                  * (foot_state_task_[i].pos
                     - model->FootPos(static_cast<message::LegName>(i)))
                + kd_foot_Cartesian_
                  * (foot_state_task_[i].vel
                     - model->FootVel(static_cast<message::LegName>(i)));

        null_space.applyOnTheRight(Eigen::MatrixXd::Identity(n_j, n_j)
                                   - jacobian.transpose() * jacobian);
        jacobian = model->FullJacob(static_cast<message::LegName>(i));

//        if(!t)
//            LOG(INFO) << null_space;

        // The first 6 elements of aq is decided completely
        // by torso acceleration.
        // So that null space for joint acceleration to operate in
        // must be orthogonal to the first 6 unit vector.
        // The above derivation indicates that the top 6 rows
        // of the null_space matrix are all zero.
        // And which we can take advantage of it.
        aq.tail<n_f>() += null_space
                * DCInv(jacobian.rightCols<n_f>() * null_space,
                        inv_m, leg_inv[i])
                * (a_cmd - model->VJDotVq(static_cast<message::LegName>(i))
                   - jacobian * aq);

//        LOG(INFO) << "round " << i + 2 << ", daq = " << daq.transpose();

        if (foot_contact_[i])
        {
            CE_.middleRows<3>(n_s + i * 3)
                    = - jacobian.leftCols<n_s>().cast<float>();
            local_jacob[i] = jacobian.rightCols<n_j - n_s>();
            res_opt.segment<3>(n_s + i * 3) = ref_force_[i].cast<float>();
        }
        else // Disable foot force if foot i is not in contact.
        {
            CE_.middleRows<3>(n_s + i * 3).setZero();
            res_opt.segment<3>(n_s + i * 3).setZero();
        }
    }

    // compensate gravity
    aq.segment<3>(3) += torso_state.rot.conjugate() * gravity_;
    res_opt.head<n_s>() = aq.head<n_s>().cast<float>();
    Eigen::VectorXf res0 = res_opt;
    g0_ = - res_opt.cwiseProduct(G_.diagonal());
    ce0_ = bias.head<n_s>().cast<float>()
            + mass.topRightCorner<n_s, n_j - n_s>().cast<float>()
              * aq.tail<n_j - n_s>().cast<float>();

    /*float opt_res = */optimization::SolveQuadProg(
                G_, g0_, CE_, ce0_, CI_, ci0_, res_opt);

//    if(!t){
//        //    LOG(INFO) << "error: " << opt_res;
//        LOG(INFO) << "ref: " << aq.transpose();
//        LOG(INFO) << "G: " << std::endl << G_;
//        LOG(INFO) << "g0: " << g0_.transpose();
//        LOG(INFO) << "CE: " << std::endl << CE_;
//        LOG(INFO) << "ce0: " << ce0_.transpose();
//        LOG(INFO) << "CI: " << std::endl << CI_;
//        LOG(INFO) << "ci0: " << ci0_.transpose();
//        //    LOG(INFO) << "acceleration: " << res_opt.segment<6>(0).transpose();
//        //    LOG(INFO) << "forces: " << res_opt.segment<12>(6).transpose();
//        t = true;
//    }

    // tau = H * aq + C - J.transpose() * f_foot
    Eigen::VectorXd torq
            = mass.bottomLeftCorner<n_j - n_s, n_s>()
              * res_opt.head<n_s>().cast<double>()
            + mass.bottomRightCorner<n_j - n_s, n_j - n_s>()
              * aq.tail<n_j - n_s>()
            + bias.tail<n_j - n_s>();

    for (int i = 0; i < 4; i++)
    {
        if (foot_contact_[i])
            torq.noalias() -= local_jacob[i].transpose()
                    * res_opt.segment<3>(n_s + i * 3).cast<double>();
    }

    torq -= model->Friction();

    for (int i = 0; i < 4; i++)
    {
        if (leg_inv[i])
        {
            cmd_->at(i * 3    ).torq = torq(i * 3    );
            cmd_->at(i * 3 + 1).torq = torq(i * 3 + 1);
            cmd_->at(i * 3 + 2).torq = torq(i * 3 + 2);
//            LOG(INFO) << "wbc used at leg " << i;
        }
        else
        {
            cmd_->at(i * 3    ).torq = 0;
            cmd_->at(i * 3 + 1).torq = 0;
            cmd_->at(i * 3 + 2).torq = 0;
        }
    }

//    LOG(INFO) << "force x: "
//              << res_opt(n_s + 0) + res_opt(n_s + 3)
//                 + res_opt(n_s + 6) + res_opt(n_s + 9);

//    LOG(INFO) << "desired acc x: " << aq(3);
//    LOG(INFO) << "actual  acc x: " << res_opt(3);
}

} /* control */

} /* dog_control */
