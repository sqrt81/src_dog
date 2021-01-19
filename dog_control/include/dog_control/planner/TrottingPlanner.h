#ifndef DOG_CONTROL_PLANNER_TROTTINGPLANNER_H
#define DOG_CONTROL_PLANNER_TROTTINGPLANNER_H

#include "dog_control/utils/ClassDeclare.h"

#include "dog_control/utils/ParamDict.h"

#include <Eigen/Eigen>

#include <boost/weak_ptr.hpp>

namespace dog_control
{

namespace planner
{

class TrottingPlanner
{
public:
    TrottingPlanner() = default;

    void Initialize(utils::ParamDictCRef dict);

    void ConnectTraj(boost::shared_ptr<control::TrajectoryController> traj);
    void ConnectClock(boost::shared_ptr<hardware::ClockBase> clock);
    void ConnectEstimator(boost::shared_ptr<estimator::EstimatorBase> estimator);
    void ConnectModel(boost::shared_ptr<physics::DogModel> model);

    void SetDesiredVel(Eigen::Vector3d vel);
    void SetSmoothingFactor(double factor);

    void Update();

private:

    boost::weak_ptr<control::TrajectoryController> traj_ptr_;
    boost::weak_ptr<hardware::ClockBase> clock_ptr_;
    boost::weak_ptr<estimator::EstimatorBase> estimator_;
    boost::weak_ptr<physics::DogModel> model_;

    double foot_x_;
    double foot_y_;
    double kd_;
    double step_period_;
    double step_stance_period_;
    double factor_;
    Eigen::Vector3d velocity_;
    double torso_height_;
    double step_height_;

    Eigen::Vector3d filtered_vel_;
    bool fl_br_swing_;
    double last_step_time_;
};

} /* planner */

} /* dog_control */

#endif /* DOG_CONTROL_PLANNER_TROTTINGPLANNER_H */
