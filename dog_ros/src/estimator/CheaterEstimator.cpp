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
    cheater_state_sub_ = nh_.subscribe(
                "/gazebo/model_states", 1,
                &CheaterEstimator::CheatedStateSub, this);
    cheater_link_sub_ = nh_.subscribe(
                "/gazebo/link_states", 1,
                &CheaterEstimator::CheatedLinkSub, this);

    cheater_contact_sub_[0] = nh_.subscribe(
                "/ground_truth/fl_foot_contact", 1,
                &CheaterEstimator::CheatedFLContactSub, this);
    cheater_contact_sub_[1] = nh_.subscribe(
                "/ground_truth/fr_foot_contact", 1,
                &CheaterEstimator::CheatedFRContactSub, this);
    cheater_contact_sub_[2] = nh_.subscribe(
                "/ground_truth/bl_foot_contact", 1,
                &CheaterEstimator::CheatedBLContactSub, this);
    cheater_contact_sub_[3] = nh_.subscribe(
                "/ground_truth/br_foot_contact", 1,
                &CheaterEstimator::CheatedBRContactSub, this);
}

void CheaterEstimator::Initialize(utils::ParamDictCRef dict)
{
    (void) dict;
}

void CheaterEstimator::Update()
{

}

void CheaterEstimator::CheatedStateSub(const gazebo_msgs::ModelStates &msg)
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

void CheaterEstimator::CheatedLinkSub(const gazebo_msgs::LinkStates &msg)
{
    Eigen::Quaterniond base_rot = res_.orientation.conjugate();

    for (unsigned int i = 0; i < msg.name.size(); i++)
    {
        if (msg.name[i] == "dog::fl_shin")
        {
            tf::quaternionMsgToEigen(msg.pose[i].orientation,
                                     ee_orientation_[0]);
            ee_orientation_[0] = base_rot * ee_orientation_[0];
        }
        else if (msg.name[i] == "dog::fr_shin")
        {
            tf::quaternionMsgToEigen(msg.pose[i].orientation,
                                     ee_orientation_[1]);
            ee_orientation_[1] = base_rot * ee_orientation_[1];
        }
        else if (msg.name[i] == "dog::bl_shin")
        {
            tf::quaternionMsgToEigen(msg.pose[i].orientation,
                                     ee_orientation_[2]);
            ee_orientation_[2] = base_rot * ee_orientation_[2];
        }
        else if (msg.name[i] == "dog::br_shin")
        {
            tf::quaternionMsgToEigen(msg.pose[i].orientation,
                                     ee_orientation_[3]);
            ee_orientation_[3] = base_rot * ee_orientation_[3];
        }
    }
}

void CheaterEstimator::CheatedFLContactSub(
        const gazebo_msgs::ContactsState &msg)
{
    Eigen::Vector3d force_total = Eigen::Vector3d::Zero();

    for (const gazebo_msgs::ContactState& item : msg.states)
    {
        Eigen::Vector3d force_inc;
        tf::vectorMsgToEigen(item.total_wrench.force, force_inc);
        force_total += force_inc;
    }

    if (msg.states.size() != 0)
        force_total /= msg.states.size();

    res_.foot_contact[0] = force_total.norm() > 1.;
    res_.foot_force[0] = ee_orientation_[0] * force_total;
}

void CheaterEstimator::CheatedFRContactSub(
        const gazebo_msgs::ContactsState &msg)
{
    Eigen::Vector3d force_total = Eigen::Vector3d::Zero();

    for (const gazebo_msgs::ContactState& item : msg.states)
    {
        Eigen::Vector3d force_inc;
        tf::vectorMsgToEigen(item.total_wrench.force, force_inc);
        force_total += force_inc;
    }

    if (msg.states.size() != 0)
        force_total /= msg.states.size();

    res_.foot_contact[1] = force_total.norm() > 1.;
    res_.foot_force[1] = ee_orientation_[1] * force_total;
}

void CheaterEstimator::CheatedBLContactSub(
        const gazebo_msgs::ContactsState &msg)
{
    Eigen::Vector3d force_total = Eigen::Vector3d::Zero();

    for (const gazebo_msgs::ContactState& item : msg.states)
    {
        Eigen::Vector3d force_inc;
        tf::vectorMsgToEigen(item.total_wrench.force, force_inc);
        force_total += force_inc;
    }

    if (msg.states.size() != 0)
        force_total /= msg.states.size();

    res_.foot_contact[2] = force_total.norm() > 1.;
    res_.foot_force[2] = ee_orientation_[2] * force_total;
}

void CheaterEstimator::CheatedBRContactSub(
        const gazebo_msgs::ContactsState &msg)
{
    Eigen::Vector3d force_total = Eigen::Vector3d::Zero();

    for (const gazebo_msgs::ContactState& item : msg.states)
    {
        Eigen::Vector3d force_inc;
        tf::vectorMsgToEigen(item.total_wrench.force, force_inc);
        force_total += force_inc;
    }

    if (msg.states.size() != 0)
        force_total /= msg.states.size();

    res_.foot_contact[3] = force_total.norm() > 1.;
    res_.foot_force[3] = ee_orientation_[3] * force_total;
}

} /* estimator */

} /* dog_control */
