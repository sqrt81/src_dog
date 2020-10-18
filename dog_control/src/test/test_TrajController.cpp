#include "dog_control/control/TrajectoryController.h"
#include "dog_control/hardware/ClockBase.h"
#include "dog_control/hardware/SimulatedHardware.h"
#include "dog_control/utils/Initializer.h"
#include "dog_control/utils/MiniLog.h"

using namespace dog_control;

int test_TrajController(int argc, char** argv)
{
    ros::init(argc, argv, "~");
    ros::Time::init();
    std::string config_file;
    ros::param::get("~config_file", config_file);

    utils::ParamDict dict = utils::BuildParamDict(config_file);

    control::TrajectoryController traj_ctrl;
    boost::shared_ptr<hardware::ClockBase> clock(
                new hardware::SimulatedClock());

    traj_ctrl.ConnectClock(clock);
    traj_ctrl.Initialize(dict);

    clock->Update();

    control::TrajectoryController::TorsoTraj traj(4);

    {
        traj[0].stamp = 0.05;
        traj[0].state.trans = {sin(0.5), cos(0.5), 1};
        traj[0].state.rot = Eigen::AngleAxisd(- 0.5,
                                              Eigen::Vector3d::UnitZ());
        traj[0].state.linear_vel = {1, 0, 0};
        traj[0].state.rot_vel = {0, 0, 0};

        traj[1].stamp = 0.1;
        traj[1].state.trans = {sin(0.5), cos(0.5), 1};
        traj[1].state.rot = Eigen::AngleAxisd(- 0.5,
                                              Eigen::Vector3d::UnitZ());
        traj[1].state.linear_vel = {1, 0, 0};
        traj[1].state.rot_vel = {0.5, 0, 0};

        traj[2].stamp = 1.1;
        traj[2].state.trans = {sin(0.5 + 0.5), cos(0.5 + 0.5), 1};
        traj[2].state.rot = Eigen::AngleAxisd(- 0.5 - 0.5,
                                              Eigen::Vector3d::UnitZ());
        traj[2].state.linear_vel = {1, 0, 0};
        traj[2].state.rot_vel = {- 0.5, 0, 0};

        traj[3].stamp = 1.15;
        traj[3].state.trans = {sin(0.5 + 0.5), cos(0.5 + 0.5), 1};
        traj[3].state.rot = Eigen::AngleAxisd(- 0.5 - 0.5,
                                              Eigen::Vector3d::UnitZ());
        traj[3].state.linear_vel = {1, 0, 0};
        traj[3].state.rot_vel = {0, 0, 0};
    }

    traj_ctrl.SetTorsoTrajectory(traj);
//    traj_ctrl.Update();

    for (int i = 0; i < 23; i++)
    {
        double t = 0.05 + i * 0.05;
        auto res = traj_ctrl.GetTorsoState(t);

        LOG(INFO) << "iter " << i << " time " << t;
        LOG(INFO) << "res: " << res.state.rot.coeffs().transpose();
        LOG(INFO) << "vel: " << res.state.rot_vel.transpose();
        LOG(INFO) << "acc: " << res.rot_acc.transpose() << std::endl;
    }

    return 0;
}
