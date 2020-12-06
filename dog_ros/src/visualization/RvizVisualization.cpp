#include "dog_ros/visualization/RvizVisualization.h"

#include <geometry_msgs/TransformStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

namespace dog_control
{

namespace visualization
{

RvizVisualization::RvizVisualization()
 : data_(nullptr)
{
    nh_ = ros::NodeHandle("~");

    est_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
                "estimated_pose", 1, false);
    torso_traj_pub_ = nh_.advertise<nav_msgs::Path>("torso_traj", 1, false);
    foot_traj_pub_[0] = nh_.advertise<nav_msgs::Path>("fl_traj", 1, false);
    foot_traj_pub_[1] = nh_.advertise<nav_msgs::Path>("fr_traj", 1, false);
    foot_traj_pub_[2] = nh_.advertise<nav_msgs::Path>("bl_traj", 1, false);
    foot_traj_pub_[3] = nh_.advertise<nav_msgs::Path>("br_traj", 1, false);
    foot_force_pub_ = nh_.advertise<visualization_msgs::Marker>(
                "foot_force", 1, false);
}

void RvizVisualization::SetData(const VisualData &data)
{
    data_ = &data;
}

void RvizVisualization::Update()
{
    if (!data_)
        return;

    const ros::Time t = ros::Time::now();

    // estimated pose
    geometry_msgs::PoseStamped tf_est;
    tf_est.header.stamp = t;
    tf_est.header.frame_id = "world";
    tf::pointEigenToMsg(data_->cur_pose.trans, tf_est.pose.position);
    tf::quaternionEigenToMsg(data_->cur_pose.rot, tf_est.pose.orientation);
    est_pos_pub_.publish(tf_est);

    // body trajectory
    if (!data_->torso_traj.empty())
    {
        nav_msgs::Path traj;
        geometry_msgs::PoseStamped pose;

        traj.header.stamp = t;
        traj.header.frame_id = "world";
        pose.header.stamp = t;
        pose.header.frame_id = traj.header.frame_id;

        for (const auto &item : data_->torso_traj)
        {
            pose.header.stamp += ros::Duration(0.001);
            tf::pointEigenToMsg(item.trans, pose.pose.position);
            tf::quaternionEigenToMsg(item.rot, pose.pose.orientation);
            traj.poses.push_back(pose);
        }

        torso_traj_pub_.publish(traj);
    }

    // foot traj
    for (int i = 0; i < 4; i++)
    {
        if (!data_->foot_traj.empty())
        {
            nav_msgs::Path traj;
            geometry_msgs::PoseStamped pose;

            traj.header.stamp = t;
            traj.header.frame_id = "torso";
            pose.header.stamp = t;
            pose.header.frame_id = traj.header.frame_id;

            for (const auto &item : data_->torso_traj)
            {
                pose.header.stamp += ros::Duration(0.001);
                tf::pointEigenToMsg(item.trans, pose.pose.position);
                tf::quaternionEigenToMsg(item.rot, pose.pose.orientation);
                traj.poses.push_back(pose);
            }

            foot_traj_pub_[i].publish(traj);
        }
    }

    // foot forces
    for (int i = 0; i < 4; i++)
    {
//        if (data_->foot_force[i].norm() > 0.1)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "torso";
            marker.header.stamp = ros::Time();
            marker.ns = "foot_force";
            marker.id = i;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::MODIFY;
            tf::pointEigenToMsg(data_->foot_local_pos[i],
                                marker.pose.position);
            tf::quaternionEigenToMsg(Eigen::Quaterniond::FromTwoVectors(
                                         Eigen::Vector3d::UnitX(),
                                         data_->foot_force[i]).normalized(),
                                     marker.pose.orientation);
            marker.scale.x = data_->foot_force[i].norm() * 0.01;
            marker.scale.y = 0.01;
            marker.scale.z = 0.01;
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;

            foot_force_pub_.publish(marker);
        }
    }
}

} /* visualization */

} /* dog_control */
