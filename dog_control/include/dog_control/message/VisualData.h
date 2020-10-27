#ifndef DOG_CONTROL_MESSAGE_VISUALDATA_H
#define DOG_CONTROL_MESSAGE_VISUALDATA_H

#include "FootState.h"
#include "ModelJointState.h"

#include <array>
#include <vector>

namespace dog_control
{

namespace message
{

/**
 * @brief The VisualData struct
 * This struct contains data for visualization.
 * It should include:
 *  estimated current body pose
 *  desired body trajectory
 *  desired swing foot trajectories
 *  foot forces
 */
struct VisualData
{
    FloatingBaseState cur_pose;

    std::vector<FloatingBaseState> torso_traj;
    std::array<std::vector<FootState>, 4> foot_traj;

    std::array<Eigen::Vector3d, 4> foot_force;
    std::array<Eigen::Vector3d, 4> foot_local_pos;
};

} /* message */

} /* dog_control */

#endif /* DOG_CONTROL_MESSAGE_VISUALDATA_H */
