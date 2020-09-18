#ifndef GAZEBO_PVE_CONTROLLER_H
#define GAZEBO_PVE_CONTROLLER_H

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

namespace effort_controllers
{

class GazeboPVEController : public controller_interface::Controller
        <hardware_interface::EffortJointInterface>
{
public:
    GazeboPVEController() = default;

    ~GazeboPVEController();

    bool init(hardware_interface::EffortJointInterface *hw,
              ros::NodeHandle &nh) override;

    void update(const ros::Time& time, const ros::Duration& period);

private:
    void commandCB(const std_msgs::Float64MultiArrayConstPtr& msg);

    struct MotorCommand
    {
        double x_desired;
        double kp;
        double v_desired;
        double kd;
        double torque;
    };

    struct MotorState
    {
        double pos;
        double vel;
        double torq;
    };

    bool publish_state_;
    ros::Publisher motor_state_pub_;
    ros::Subscriber command_sub_;

    unsigned int joint_cnt_;
    std::vector<hardware_interface::JointHandle> joints_;
    std::vector<MotorCommand> joint_cmd_;
    std::vector<MotorState> joint_state_;
    std::vector<std::array<double, 2>> joint_torque_limit_;
};

}

#endif // GAZEBO_PVE_CONTROLLER_H
