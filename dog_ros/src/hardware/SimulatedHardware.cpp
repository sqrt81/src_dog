#include "dog_ros/hardware/SimulatedHardware.h"
#include "dog_control/utils/Math.h"

#include <eigen_conversions/eigen_msg.h>

namespace dog_control
{

namespace hardware
{

SimulatedHardware::SimulatedHardware()
{
    nh_ = ros::NodeHandle("~");
    imu_sub_ = nh_.subscribe(
                "imu", 1, &SimulatedHardware::ImuCb, this);
    joint_state_sub_ = nh_.subscribe(
                "joint_state", 1, &SimulatedHardware::JointStateCb, this);
    joint_cmd_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(
                "joint_cmd", 1, false);
}

SimulatedHardware::~SimulatedHardware()
{

}

void SimulatedHardware::Initialize(utils::ParamDictCRef dict)
{
    // different from usual initializing,
    // this method grasp all keys in namespace "MotorName"
    // and stores them in motor_index_.

    const std::string ns_name = std::string("motor_name/");
    const int ns_len = ns_name.size();

    for (const std::pair<std::string, double>& iter : dict)
    {
        if (iter.first.substr(0, ns_len) == ns_name)
        {
            const std::string motor_name = iter.first.substr(ns_len);

            if (motor_name.find('/') == std::string::npos)
            {
                // the motor_name does not contains a sub-namespace name
                motor_index_.insert(std::make_pair(
                                        motor_name,
                                        utils::round(iter.second)));
            }
        }
    }
}

void SimulatedHardware::Spin()
{
    ros::spin();
}

void SimulatedHardware::PublishCommand(MotorCommandCRef command)
{
    std_msgs::Float64MultiArray cmd_array;
    cmd_array.data.resize(60);

    for (unsigned int i = 0; i < 12; i++)
    {
        cmd_array.data[     i] = command[i].x_desired;
        cmd_array.data[12 + i] = command[i].kp;
        cmd_array.data[24 + i] = command[i].v_desired;
        cmd_array.data[36 + i] = command[i].kd;
        cmd_array.data[48 + i] = command[i].torq;
    }

    joint_cmd_pub_.publish(cmd_array);
}

void SimulatedHardware::ImuCb(sensor_msgs::ImuConstPtr imu)
{
    imu_info_.stamp = imu->header.stamp.toSec();
    tf::quaternionMsgToEigen(imu->orientation, imu_info_.imu_data.rotation);
    imu_info_.imu_data.rotation.normalize();
    tf::vectorMsgToEigen(imu->angular_velocity,
                         imu_info_.imu_data.angular_speed);
    tf::vectorMsgToEigen(imu->linear_acceleration,
                         imu_info_.imu_data.acceleration);
}

void SimulatedHardware::JointStateCb(sensor_msgs::JointStateConstPtr js)
{
    joint_state_.stamp = js->header.stamp.toSec();

    if (js->name.size() != 12
            || js->position.size() != 12
            || js->velocity.size() != 12
            || js->effort.size() != 12)
    {
        ROS_ERROR("Simulated Hardware: joint states with wrong size");
        return;
    }

    for (unsigned int i = 0; i < 12; i++)
    {
        const auto iter = motor_index_.find(js->name[i]);

        if (iter == motor_index_.end())
        {
            ROS_ERROR("Simulated Hardware: unknown joint name : %s",
                      js->name[i].c_str());
        }
        else
        {
            message::SingleJointState& state
                    = joint_state_.joint_state[iter->second];

            state.vel = (js->position[i] - state.pos) * 1000;
            state.pos = js->position[i];
//            state.vel = js->velocity[i];
            state.eff = js->effort[i];
        }
    }

}

} /* hardware */

} /* dog_control */
