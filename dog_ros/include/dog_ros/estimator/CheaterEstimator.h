#ifndef DOG_CONTROL_ESTIMATOR_CHEATERESTIMATOR_H
#define DOG_CONTROL_ESTIMATOR_CHEATERESTIMATOR_H

#include "dog_control/estimator/EstimatorBase.h"

#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>

namespace dog_control
{

namespace estimator
{

class CheaterEstimator : public EstimatorBase
{
public:
    CheaterEstimator();

    void Initialize(utils::ParamDictCRef dict) override;

    void Update() override;

    void WriteResult(EstimatorResult &result) const override;

private:
    void CheatedMsgSub(const gazebo_msgs::ModelStates& msg);
    ros::NodeHandle nh_;
    ros::Subscriber cheater_sub_;
};

} /* estimator */

} /* dog_control */

#endif /* DOG_CONTROL_ESTIMATOR_CHEATERESTIMATOR_H */
