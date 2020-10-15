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

    control::TrajectoryController::TorsoTraj traj(20);

    for (int i = 0; i < 20; i++)
    {
        traj[i].stamp = clock->Time() + i * 0.01;

        traj[i].state.trans = {sin(0.01 * i + 0.5), cos(0.01 * i + 0.5), 1};
        traj[i].state.rot = Eigen::AngleAxisd(- 0.01 * i - 0.5,
                                              Eigen::Vector3d::UnitZ());
        traj[i].state.linear_vel = {1, 0, 0};
        traj[i].state.rot_vel = {0, 0, - 1};
    }

    traj_ctrl.SetTorsoTrajectory(traj);
    traj_ctrl.Update();

    for (int i = 0; i < 20; i++)
    {
        double t = clock->Time() + 0.005 + i * 0.01;
        auto res = traj_ctrl.GetTorsoPose(t);

        LOG(INFO) << "iter " << i << " time " << t;
        LOG(INFO) << "res: " << res.state.trans.transpose();
        LOG(INFO) << "vel: " << res.state.linear_vel.transpose();
        LOG(INFO) << "acc: " << res.linear_acc.transpose();
    }
}
