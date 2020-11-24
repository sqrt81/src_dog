#ifndef DOG_CONTROL_CONTROL_FOOTSWINGTRAJBASE_H
#define DOG_CONTROL_CONTROL_FOOTSWINGTRAJBASE_H

#include <Eigen/Eigen>

#include "dog_control/message/LegConfiguration.h"

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
                        message::LegConfigRef conf) const = 0;

    /**
     * @brief DecideEnd
     * Decide wheter to end this trajectory now
     * and change this foot into stance state.
     * @param t                 current time
     * @param in_contact        if the foot is currently in contact
     * @return                  true if this trajectory should end
     */
    virtual bool DecideEnd(double t, bool in_contact) = 0;

protected:
    double begin_time_;
    double end_time_;
};

} /* control */

} /* dog_control */

#endif /* DOG_CONTROL_CONTROL_FOOTSWINGTRAJBASE_H */
