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
    void ConnectFoot(boost::shared_ptr<control::FootPosController> foot_pos);
    void ConnectMPC(boost::shared_ptr<control::MPCBase> mpc);
    void ConnectClock(boost::shared_ptr<hardware::ClockBase> clock);
    void ConnectModel(boost::shared_ptr<physics::DogModel> model);
    void GetDict(utils::ParamDictCRef dict);

    void Update();

private:
    void ReWriteDictPos();
    void ReWriteDictVel();

    boost::weak_ptr<control::TrajectoryController> traj_ptr_;
    boost::weak_ptr<control::FootPosController> foot_pos_ptr_;
    boost::weak_ptr<control::MPCBase> mpc_ptr_;
    boost::weak_ptr<hardware::ClockBase> clock_ptr_;
    boost::weak_ptr<physics::DogModel> model_ptr_;

    utils::ParamDict dict_;

    double forward_vel_;    // velocity when in air
    double step_width_;     // distance between two stance feet
    double flip_time_;      // duration for one flip
    double aerial_time_;    // time in air
    double stance_time_;    // time on ground
    double torso_height_;   // torso height during stance stage
    double gravity_;        // gravity's z component, positive
    double stable_time_;    // time for stablization after landing
    double rot_factor_up_;
    double rot_factor_down_;

    bool airborn_;          // whether the robot is in air
    bool upside_down_;      // whether the robot is flipped over
    bool flip_done_;        // whether the robot has finished flipping
    double last_update_time_;
    double contact_time_;   // when did the robot land
    bool contact_[4];
    double yaw_;             // torso yaw
};

} /* planner */

} /* dog_control */

#endif /* DOG_CONTROL_PLANNER_HALFFLIPPLANNER_H */
