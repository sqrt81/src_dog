#ifndef DOG_CONTROL_CONTROL_ConfSpaceTraj_H
#define DOG_CONTROL_CONTROL_ConfSpaceTraj_H

#include "dog_control/utils/ClassDeclare.h"

#include "FootSwingTrajBase.h"

#include "dog_control/message/FootState.h"
#include "dog_control/utils/CubicSpline.h"

namespace dog_control
{

namespace control
{

class ConfSpaceTraj : public FootSwingTrajBase
{
public:
    ConfSpaceTraj(double beg_t,
                  double end_t,
                  const Eigen::Vector3d &beg_pos,
                  const Eigen::Vector3d &beg_vel,
                  message::LegConfigCRef beg_conf,
                  const Eigen::Vector3d &end_pos,
                  const Eigen::Vector3d &end_vel,
                  message::LegConfigCRef end_conf,
                  const physics::DogModel &model);

    void Replan(double replan_t,
                const Eigen::Vector3d &end_pos,
                const Eigen::Vector3d &end_vel);

    void Sample(double t,
                Eigen::Vector3d &local_pos,
                Eigen::Vector3d &local_vel,
                Eigen::Vector3d &local_acc,
                bool &hip_outwards,
                bool &knee_outwards) const override;

private:
    double half_time_;
    const physics::DogModel &model_;
    const message::LegName leg_;

    utils::CubicSpline<Eigen::Vector3d> joint_traj_;
};

} /* control */

} /* dog_control */

#endif /* DOG_CONTROL_CONTROL_ConfSpaceTraj_H */
