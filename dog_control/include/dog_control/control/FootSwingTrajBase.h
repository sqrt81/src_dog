#ifndef DOG_CONTROL_CONTROL_FOOTSWINGTRAJBASE_H
#define DOG_CONTROL_CONTROL_FOOTSWINGTRAJBASE_H

#include <Eigen/Eigen>

namespace dog_control
{

namespace control
{

class FootSwingTrajBase
{
public:
    FootSwingTrajBase() = default;

    double BeginTime() const
    {
        return begin_time_;
    }

    double EndTime() const
    {
        return end_time_;
    }

    virtual void Sample(double t,
                        Eigen::Vector3d &local_pos,
                        Eigen::Vector3d &local_vel,
                        Eigen::Vector3d &local_acc,
                        bool &hip_outwards,
                        bool &knee_outwards) const = 0;

protected:
    double begin_time_;
    double end_time_;
};

} /* control */

} /* dog_control */

#endif /* DOG_CONTROL_CONTROL_FOOTSWINGTRAJBASE_H */
