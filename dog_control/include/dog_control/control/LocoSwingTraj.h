#ifndef DOG_CONTROL_CONTROL_LOCOSWINGTRAJ_H
#define DOG_CONTROL_CONTROL_LOCOSWINGTRAJ_H

#include "FootSwingTrajBase.h"

#include "dog_control/utils/CubicSpline.h"

namespace dog_control
{

namespace control
{

class LocoSwingTraj : public FootSwingTrajBase
{
public:
    LocoSwingTraj(double beg_t,
                  double end_t,
                  const Eigen::Vector3d &beg_pos,
                  const Eigen::Vector3d &beg_vel,
                  const Eigen::Vector3d &end_pos,
                  const Eigen::Vector3d &end_vel,
                  const Eigen::Vector3d &raise_offset);

    void Replan(double replan_t,
                const Eigen::Vector3d &end_pos,
                const Eigen::Vector3d &end_vel);

    void Sample(double t,
                Eigen::Vector3d &local_pos,
                Eigen::Vector3d &local_vel,
                Eigen::Vector3d &local_acc,
                message::LegConfigRef conf) const override;

    bool DecideEnd(double t, bool in_contact) override;

private:
    double half_time_;
    Eigen::Vector3d raise_offset_;

    utils::CubicSpline<Eigen::Vector3d> rise_;
    utils::CubicSpline<Eigen::Vector3d> fall_;
};

} /* control */

} /* dog_control */

#endif /* DOG_CONTROL_CONTROL_LOCOSWINGTRAJ_H */
