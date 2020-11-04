#ifndef DOG_ROS_ESTIMATOR_CHEATERESTIMATOR_H
#define DOG_ROS_ESTIMATOR_CHEATERESTIMATOR_H

#include "dog_control/estimator/EstimatorBase.h"

#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/ContactsState.h>

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

private:
    void CheatedStateSub(const gazebo_msgs::ModelStates& msg);
    void CheatedLinkSub(const gazebo_msgs::LinkStates& msg);

    void CheatedFLContactSub(const gazebo_msgs::ContactsState& msg);
    void CheatedFRContactSub(const gazebo_msgs::ContactsState& msg);
    void CheatedBLContactSub(const gazebo_msgs::ContactsState& msg);
    void CheatedBRContactSub(const gazebo_msgs::ContactsState& msg);

    ros::NodeHandle nh_;
    ros::Subscriber cheater_state_sub_;
    ros::Subscriber cheater_link_sub_;
    std::array<Eigen::Quaterniond, 4> ee_orientation_;
    std::array<ros::Subscriber, 4> cheater_contact_sub_;
};

} /* estimator */

} /* dog_control */

#endif /* DOG_ROS_ESTIMATOR_CHEATERESTIMATOR_H */
