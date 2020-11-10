#include "dog_control/control/LocoSwingTraj.h"
#include "dog_control/utils/MiniLog.h"

#include <Eigen/Eigen>

using namespace dog_control;

int test_SwingTraj()
{
    double t1 = 0.1;
    double t2 = 0.3;

    Eigen::Vector3d beg_pos(0, 0, 0), end_pos(0.1, 0.3, 0);
    Eigen::Vector3d raise(0, 0, 0.05);
    Eigen::Vector3d beg_vel(- 0.5, - 1.5, 0.1), end_vel(- 0.5, - 1.5, - 0.1);

    control::LocoSwingTraj traj(
                t1, t2, beg_pos, beg_vel, end_pos, end_vel, raise);

    control::FootSwingTrajBase* ptr = &traj;

    LOG(INFO) << "beg time: " << ptr->BeginTime()
              << " end time: " << ptr->EndTime();

    for (int i = 0; i < 100; i++)
    {
        Eigen::Vector3d p, v, a;
        bool leg_conf[2];
        ptr->Sample(0.075 + i * 0.0025, p, v, a, leg_conf[0], leg_conf[1]);
        LOG(INFO) << "time : " << 0.075 + i * 0.0025;
        LOG(INFO) << "p: " << p.transpose() << std::endl
                  << " v: " << v.transpose() << std::endl
                  << " a: " << a.transpose() << std::endl;
    }

    return 0;
}
