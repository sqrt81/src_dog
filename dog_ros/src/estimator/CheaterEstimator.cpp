#include "dog_ros/estimator/CheaterEstimator.h"

#include "dog_control/utils/MiniLog.h"

#include <eigen_conversions/eigen_msg.h>

namespace dog_control
{

namespace estimator
{

CheaterEstimator::CheaterEstimator()
 : EstimatorBase(), nh_("~")
{
    cheater_sub_ = nh_.subscribe("/gazebo/model_states", 1,
                                 &CheaterEstimator::CheatedMsgSub, this);
}

void CheaterEstimator::Initialize(utils::ParamDictCRef dict)
{
    (void) dict;
}

void CheaterEstimator::Update()
{

}

void CheaterEstimator::WriteResult(EstimatorResult &result) const
{
    result = res_;
}

void CheaterEstimator::CheatedMsgSub(const gazebo_msgs::ModelStates &msg)
{
    for (size_t i = 0; i < msg.name.size(); i++)
    {
        if (msg.name[i] == "dog")
        {
            tf::pointMsgToEigen(msg.pose[i].position, res_.position);
            tf::quaternionMsgToEigen(msg.pose[i].orientation,
                                     res_.orientation);
            tf::vectorMsgToEigen(msg.twist[i].linear, res_.linear_vel);
            tf::vectorMsgToEigen(msg.twist[i].angular, res_.rot_vel);

            // change global velocity to local frame
            res_.linear_vel = res_.orientation.conjugate() * res_.linear_vel;
            res_.rot_vel = res_.orientation.conjugate() * res_.rot_vel;
        }
    }
}

} /* estimator */

} /* dog_control */
