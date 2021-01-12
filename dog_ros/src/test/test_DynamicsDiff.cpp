#include "dog_ros/test_lists.h"

#include "dog_control/physics/SimDogModel.h"
#include "dog_control/utils/MiniLog.h"
#include "dog_control/utils/Initializer.h"
#include "dog_control/physics/EigenToolbox.h"
#include "dog_control/physics/SpatialToolbox.h"

#include <ros/ros.h>

using namespace dog_control;

Eigen::VectorXd f(physics::SimDogModel &model, const Eigen::VectorXd &tau)
{
    Eigen::MatrixXd H = model.MassMatrix();
    Eigen::VectorXd C = model.BiasForces();

    return H.inverse() * (tau - C);
}

Eigen::MatrixXd dfq(physics::SimDogModel &model, const Eigen::VectorXd &tau)
{
    Eigen::MatrixXd H = model.MassMatrix();
    Eigen::VectorXd C = model.BiasForces();
    Eigen::VectorXd f = tau - C;
    Eigen::MatrixXd H_inv = H.inverse();
    auto H_diff = model.MassMatrixDiff();
    auto C_diff = model.BiasForceDiff();

    Eigen::MatrixXd jacob
            = Eigen::MatrixXd::Zero(tau.size(), tau.size());

    for (int i = 0; i < tau.size(); i++)
    {
        if (i >= 6)
            jacob.col(i) = - H_inv * H_diff[i - 6] * H_inv * f;

        jacob.col(i) -= H_inv * (C_diff[i]);
    }

    return jacob;
}

Eigen::MatrixXd dfvq(physics::SimDogModel &model, const Eigen::VectorXd &tau)
{
    Eigen::MatrixXd H = model.MassMatrix();
    Eigen::VectorXd C = model.BiasForces();
    Eigen::VectorXd f = tau - C;
    Eigen::MatrixXd H_inv = H.inverse();
    auto H_diff = model.MassMatrixDiff();
    auto C_diff = model.BiasForceDiff();

    Eigen::MatrixXd jacob
            = Eigen::MatrixXd::Zero(tau.size(), tau.size());

    for (int i = 0; i < tau.size(); i++)
    {
        jacob.col(i) -= H_inv * (C_diff[i + tau.size()]);
    }

    return jacob;
}

Eigen::MatrixXd dft(physics::SimDogModel &model, const Eigen::VectorXd &tau)
{
    Eigen::MatrixXd H = model.MassMatrix();
    Eigen::VectorXd C = model.BiasForces();
    Eigen::VectorXd f = tau - C;
    Eigen::MatrixXd H_inv = H.inverse();

    return H_inv;
}

int test_DynamicsDiff(int argc, char** argv)
{
    ros::init(argc, argv, "~");

    std::string config_file;
    ros::param::get("~config_file", config_file);

    utils::ParamDict dict = utils::BuildParamDict(config_file);
//    const double dt = ReadParOrDie(dict,
//                                   PARAM_WITH_NS(control_period, control));
    message::FloatingBaseJointState js;

//    physics::spatial::SVec axis;
//    physics::spatial::JointType type
//            = physics::spatial::revolute;
//    axis << 0, - 2, 0, 0, 0, 0;
//    auto tf1 = physics::spatial::BuildJointTransform(axis, type, 1);
//    auto tf2 = physics::spatial::BuildJointTransform(axis, type, 1.001);
//    std::cout << (tf2 - tf1) * 1000 << std::endl << std::endl;
//    std::cout << physics::spatial::BuildJointTransformDiff(axis, type, 1)
//              << std::endl;

//    return 0;

    boost::shared_ptr<physics::SimDogModel> model(new physics::SimDogModel());
    model->Initialize(dict);
    model->ToggleDiff(true);
    const double scale = 0.00001;
//    const int joint_id = 4;

//    srand(time(nullptr));

    // randomly set joint state
//    js.base.trans = {0, 1, 0};
//    js.base.rot = {1, 2, 3, 4};
//    js.base.rot.normalize();
//    js.base.linear_vel = {0, 0, 0};
//    js.base.rot_vel = {0, 0, 1};
    js.base.trans = Eigen::Vector3d::Random();
    js.base.rot = Eigen::AngleAxisd(
                rand(), Eigen::Vector3d::Random().normalized());
    js.base.linear_vel = Eigen::Vector3d::Random();
    js.base.rot_vel = Eigen::Vector3d::Random();
    js.q.resize(12, 0);
    js.dq.resize(12, 0);
    Eigen::VectorXd tau = Eigen::VectorXd::Random(18);

    physics::spatial::SVec dif_xbase;
    physics::spatial::SVec dif_vbase;
    Eigen::VectorXd dif_q;
    Eigen::VectorXd dif_dq;
    dif_q = Eigen::VectorXd::Random(12);
    dif_dq = Eigen::VectorXd::Random(12);
//    dif_vbase(2) = 1;

    for (int i = 0; i < 12; i++)
    {
        js.q[i] = dif_q(i);
        js.dq[i] = dif_dq(i);
    }

    model->Update(js);
    auto f0 = f(*model, tau);
    auto diff_q = dfq(*model, tau);
    auto diff_vq = dfvq(*model, tau);
    auto diff_tau = dft(*model, tau);
//    auto mass_diff = model->MassMatrixDiff();
//    Eigen::MatrixXd H0 = model->MassMatrix().inverse();
//    auto V0 = model->v_[joint_id];
//    auto dVq = model->dv_dq_[joint_id];
//    auto dVvq = model->dv_dvq_[joint_id];
//    auto a0 = model->a_C_[joint_id];
//    auto daq = model->daC_dq_[joint_id];
//    auto davq = model->daC_dvq_[joint_id];
//    auto dc = model->BiasForceDiff();
//    Eigen::VectorXd C0 = model->BiasForces();
//    auto J0 = model->FullJacob(message::FR);
//    auto dJbase = model->FullJacobBaseDiff(message::FR);
//    auto dJjoint = model->FullJacobJointDiff(message::FR);

    dif_q = Eigen::VectorXd::Random(12) * scale;
    dif_dq = Eigen::VectorXd::Random(12) * scale;
//    dif_q(0) = 0.01;

    dif_xbase = physics::spatial::SVec::Random() * scale;
    dif_vbase = physics::spatial::SVec::Random() * scale;
//    dif_xbase(0) = 0.01;

    const double norm1 = std::max(dif_q.cwiseAbs().maxCoeff(),
                                  dif_dq.cwiseAbs().maxCoeff());
    const double norm2 = std::max(dif_xbase.cwiseAbs().maxCoeff(),
                                  dif_vbase.cwiseAbs().maxCoeff());
    const double norm = std::max(norm1, norm2);
    js.base.trans += dif_xbase.tail<3>();
    js.base.rot *= physics::SO3ToQuat(dif_xbase.head<3>());
    js.base.rot_vel += dif_vbase.head<3>();
    js.base.linear_vel += dif_vbase.tail<3>();

    for (int i = 0; i < 12; i++)
    {
        js.q[i] += dif_q(i);
        js.dq[i] += dif_dq(i);
    }

    Eigen::VectorXd dif_t = Eigen::VectorXd::Random(18) * scale;
    tau += dif_t;

    model->Update(js);
    auto f1 = f(*model, tau);
//    Eigen::MatrixXd H1 = model->MassMatrix().inverse();
//    auto V1 = model->v_[joint_id];
//    auto a1 = model->a_C_[joint_id];
//    Eigen::VectorXd C1 = model->BiasForces();
//    auto J1 = model->FullJacob(message::FR);

//    Eigen::MatrixXd diff_m = /*Eigen::MatrixXd::Zero(18, 18)*/H1 - H0;
//    physics::spatial::SVec diff_v = V1 - V0;
//    physics::spatial::SVec diff_a = a1 - a0;
//    Eigen::VectorXd diff_c = C0 - C1;
//    Eigen::VectorXd diff_c = Eigen::VectorXd::Zero(18);
//    physics::DogModel::FullJacobMat diff_J = J1 - J0;
    Eigen::VectorXd diff_f = f1 - f0;

//    for (int i = 0; i < 12; i++)
//    {
//        diff_v -= dVq[i + 6] * dif_q(i);
//        diff_v -= dVvq[i + 6] * dif_dq(i);
//    }

//    for (int i = 0; i < 6; i++)
//    {
//        diff_v -= dVq[i] * dif_xbase(i);
//        diff_v -= dVvq[i] * dif_vbase(i);
//    }

//    for (int i = 0; i < 12; i++)
//    {
//        diff_a -= daq[i + 6] * dif_q(i);
//        diff_a -= davq[i + 6] * dif_dq(i);
//    }

//    for (int i = 0; i < 6; i++)
//    {
//        diff_a -= daq[i] * dif_xbase(i);
//        diff_a -= davq[i] * dif_vbase(i);
//    }

//    for (int i = 0; i < 12; i++)
//        diff_m += H0 * mass_diff[i] * H0 * dif_q(i);

//    for (int i = 0; i < 12; i++)
//    {
//        diff_c += dc[i + 6] * dif_q(i);
//        diff_c += dc[i + 6 + 18] * dif_dq(i);
//    }

//    for (int i = 0; i < 6; i++)
//    {
//        diff_c += dc[i] * dif_xbase(i);
//        diff_c += dc[i + 18] * dif_vbase(i);
//    }

//    for (int i = 0; i < 3; i++)
//    {
//        diff_J -= dJbase[i] * dif_xbase(i);
//    }

//    for (int i = 0; i < 3; i++)
//    {
//        diff_J -= dJjoint[i] * dif_q(i + 3);
//    }

    for (int i = 0; i < 6; i++)
    {
        diff_f -= diff_q.col(i) * dif_xbase(i);
        diff_f -= diff_vq.col(i) * dif_vbase(i);
    }

    for (int i = 6; i < 18; i++)
    {
        diff_f -= diff_q.col(i) * dif_q(i - 6);
        diff_f -= diff_vq.col(i) * dif_dq(i - 6);
    }

    diff_f -= diff_tau * dif_t;

//    std::cout << "H diff:" << std::endl
//              << (H1 - H0).cwiseAbs().maxCoeff() / norm
//              << std::endl << std::endl;

//    std::cout << "dH:" << std::endl << diff_m.cwiseAbs().maxCoeff() / norm
//              << std::endl << std::endl;

//    std::cout << "Mass Matrix Diff err: " << std::endl
//              << diff_m.topLeftCorner(6, 6).cwiseAbs().maxCoeff() << "\t "
//              << diff_m.topRightCorner(6, 12).cwiseAbs().maxCoeff() << "\t "
//              << diff_m.bottomRightCorner(12, 12).cwiseAbs().maxCoeff()
//              << "\t " << std::endl;
//    std::cout << "Mass Matrix Diff err: " << std::endl
//              << diff_m.cwiseAbs().maxCoeff() / norm
//              << std::endl << std::endl;

//    std::cout << "v diff: " << std::endl
//              << (V1 - V0).transpose() / norm << std::endl << std::endl;
//    std::cout << "dv: " << std::endl
//              << diff_v.cwiseAbs().maxCoeff() / norm << std::endl << std::endl;

//    std::cout << "a diff: " << std::endl
//              << (a1 - a0).transpose() / norm << std::endl << std::endl;
//    std::cout << "da: " << std::endl
//              << diff_a.cwiseAbs().maxCoeff() / norm << std::endl << std::endl;

//    std::cout << "a0: " << a0.transpose() << std::endl
//              << "a1: " << a1.transpose() << std::endl;

//    std::cout << "c0: " << C0.transpose() << std::endl
//              << "c1: " << C1.transpose() << std::endl;

//    std::cout << "bias diff : " << std::endl
//              << (C1 - C0).transpose() / norm << std::endl << std::endl;
//    std::cout << "dbias : " << std::endl
//              << diff_c/*.head<6>()*/.transpose().cwiseAbs().maxCoeff() / norm
//              << std::endl << std::endl;

//    std::cout << "dJ: " << std::endl
//              << dJjoint[0] << std::endl << std::endl;
//    std::cout << "J0: " << std::endl
//              << J0 << std::endl << std::endl;
//    std::cout << "J1: " << std::endl
//              << J1 << std::endl << std::endl;

//    std::cout << "J diff : " << std::endl
//              << (J1 - J0) / norm << std::endl << std::endl;
//    std::cout << "dJ : " << std::endl
//              << diff_J.cwiseAbs().maxCoeff() / norm
//              << std::endl << std::endl;

    std::cout << "f diff : " << std::endl
              << (f1 - f0).transpose() << std::endl << std::endl;
    std::cout << "df : " << std::endl
              << (diff_f.array() / (f1 - f0).array()).cwiseAbs().transpose()
              << std::endl << std::endl;

    return 0;
}
