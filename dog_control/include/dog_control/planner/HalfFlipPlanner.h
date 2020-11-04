#ifndef DOG_CONTROL_PLANNER_HALFFLIPPLANNER_H
#define DOG_CONTROL_PLANNER_HALFFLIPPLANNER_H

#include "dog_control/utils/ClassDeclare.h"

#include "dog_control/utils/ParamDict.h"

#include <boost/weak_ptr.hpp>

namespace dog_control
{

namespace planner
{

class HalfFlipPlanner
{
public:
    HalfFlipPlanner() = default;

    void Initialize(utils::ParamDictCRef dict);

    void ConnectTraj(boost::shared_ptr<control::TrajectoryController> traj);
    void ConnectClock(boost::shared_ptr<hardware::ClockBase> clock);
    void ConnectModel(boost::shared_ptr<physics::DogModel> model);

    void Update();

private:
    boost::weak_ptr<control::TrajectoryController> traj_ptr_;
    boost::weak_ptr<hardware::ClockBase> clock_ptr_;
    boost::weak_ptr<physics::DogModel> model_ptr_;

    double forward_vel_;    // velocity when in air
    double step_width_;     // distance between two stance feet
    double flip_time_;      // duration for one flip
    double aerial_time_;    // time in air
    double stance_time_;    // time on ground
    double torso_height_;   // torso height during stance stage
    double gravity_;        // gravity's z component, positive
    double rot_factor_;

    bool upside_down_;      // whether the robot is flipped over
    double last_update_time_;
    bool contact_[4];
};

} /* planner */

} /* dog_control */

#endif /* DOG_CONTROL_PLANNER_HALFFLIPPLANNER_H */
