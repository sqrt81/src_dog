#include "../include/gazebo_pve_controller/gazebo_pve_controller.h"

#include <pluginlib/class_list_macros.hpp>
#include <urdf/model.h>

namespace effort_controllers
{

namespace
{

inline double clamp(const double x, const double lower, const double upper)
{
    if (x < lower)
        return lower;

    if (x > upper)
        return upper;

    return x;
}

const std::string param_name = "joints";
const std::string shared_controls = "mimic_joints";
const std::string shared_torq_limits = "mimic_joint_torque_limits";
const std::string shared_control_kp = "mimic_physics_kp";
const std::string shared_control_kd = "mimic_physics_kd";
const std::string pub_stat_option = "publish_states";

} // anonymous namespace

GazeboPVEController::~GazeboPVEController()
{
    if (publish_state_)
        motor_state_pub_.shutdown();

    command_sub_.shutdown();
}

bool GazeboPVEController::init(hardware_interface::EffortJointInterface *hw,
          ros::NodeHandle &nh)
{
    ROS_INFO("PVE Controller: Initing...");

    joints_.clear();

    // get controlled joints
    std::vector<std::string> joint_names;

    if (!nh.getParam(param_name, joint_names))
    {
        ROS_ERROR_STREAM("PVE Controller: Failed to getParam '"
                         << param_name << "' (namespace: "
                         << nh.getNamespace() << ").");
        return false;
    }

    joint_cnt_ = joint_names.size();

    if (joint_cnt_ == 0)
    {
        ROS_ERROR_STREAM("PVE Controller: List of joint names is empty.");
        return false;
    }

    mimicking_joints_.resize(joint_cnt_);
    std::map<std::string, std::string> mimic_joint_names;

    if (!nh.getParam(pub_stat_option, publish_state_))
    {
        ROS_INFO_STREAM("PVE Controller: '" << pub_stat_option
                        << "' is not set.");
        publish_state_ = true;
    }

    if (publish_state_)
    {
        ROS_INFO("PVE Controller: Publishing motor state"
                 " on incoming commands.");
    }

    // Get URDF
    urdf::Model urdf;

    if (!urdf.initParamWithNodeHandle("robot_description", nh))
    {
        ROS_ERROR("PVE Controller: Failed to parse urdf file");
        return false;
    }

    for (unsigned int i = 0; i < joint_cnt_; i++)
    {
        const std::string& joint_name = joint_names[i];

        ROS_INFO("PVE Controller: Reading property of joint %s",
                 joint_name.c_str());

        try
        {
            joints_.push_back(hw->getHandle(joint_name));
        }
        catch(const hardware_interface::HardwareInterfaceException& e)
        {
            ROS_ERROR_STREAM("PVE Controller: Exception thrown: " << e.what());
            return false;
        }

        urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_name);

        if (!joint_urdf)
        {
            ROS_ERROR("Could not find joint '%s' in urdf", joint_name.c_str());
            return false;
        }

        if (joint_urdf->type != urdf::Joint::REVOLUTE)
        {
            ROS_ERROR("Only revolute joint is supported!");
            return false;
        }

        joint_torque_limit_.push_back(joint_urdf->limits->effort);

    }

    if (nh.getParam(shared_controls, mimic_joint_names))
    {
        std::map<std::string, double> mimic_joint_limits;

        if (!nh.getParam(shared_torq_limits, mimic_joint_limits))
        {
            ROS_ERROR_STREAM("PVE Controller: '"
                             << shared_torq_limits
                             << "' not set while mimic is required");
            return false;
        }

        if (!nh.getParam(shared_control_kp, mimic_kp_))
        {
            ROS_ERROR_STREAM("PVE Controller: '"
                             << shared_control_kp
                             << "' not set while mimic is required");
            return false;
        }

        if (!nh.getParam(shared_control_kd, mimic_kd_))
        {
            ROS_ERROR_STREAM("PVE Controller: '"
                             << shared_control_kd
                             << "' not set while mimic is required");
            return false;
        }

        // be sure only the mimicked joint is among joint_names
        // so close chain is avoided

        for (const std::pair<std::string, std::string> &joint_pair
             : mimic_joint_names)
        {
            if (std::find(joint_names.begin(),
                          joint_names.end(),
                          joint_pair.first) != joint_names.end())
            {
                ROS_ERROR_STREAM("PVE Controller: "
                                 "The mimicking joint '" << joint_pair.first
                                 << "' is among the actuated joints.");
                return false;
            }

            const auto mimicked = std::find(joint_names.begin(),
                                            joint_names.end(),
                                            joint_pair.second);

            if (mimicked == joint_names.end())
            {
                ROS_ERROR_STREAM("PVE Controller: "
                                 "The mimicked joint '" << joint_pair.second
                                 << "' is not among the actuated joints.");
                return false;
            }

            ROS_INFO_STREAM("PVE Controller: Reading property of"
                            " mimicking joint " << joint_pair.first);
            const int mimicked_id = mimicked - joint_names.begin();

            try
            {
                mimicking_joints_[mimicked_id]
                        .push_back(hw->getHandle(joint_pair.first));
            }
            catch (const hardware_interface::HardwareInterfaceException& e)
            {
                ROS_ERROR_STREAM("PVE Controller: Exception thrown: "
                                 << e.what());
                return false;
            }

            // overwrite mimicked joint's limits
            const auto limit_iter = mimic_joint_limits.find(joint_pair.second);

            if (limit_iter == mimic_joint_limits.end())
            {
                ROS_ERROR_STREAM("PVE Controller: "
                                 "Torque limit of mimicked joint '"
                                 << joint_pair.second << "' is not set.");
                return false;
            }

            joint_torque_limit_[mimicked_id] = limit_iter->second;
        }
    }

    joint_cmd_.resize(joint_cnt_, {0., 0., 0., 0., 0.});
    joint_state_.resize(joint_cnt_, {0., 0., 0.});

    if (publish_state_)
    {
        motor_state_pub_ = nh.advertise<std_msgs::Float64MultiArray>(
                    "motor_states", 1, false);
    }

    command_sub_ = nh.subscribe<std_msgs::Float64MultiArray>(
                "command", 1, &GazeboPVEController::commandCB, this);

    return true;
}

void GazeboPVEController::update(const ros::Time &time,
                                 const ros::Duration &period)
{
    // pretend to use the inputs
    (void) time;
    (void) period;

    for (unsigned int i = 0; i < joint_cnt_; i++)
    {
        hardware_interface::JointHandle& motor = joints_[i];
        const MotorCommand& motor_cmd = joint_cmd_[i];
        MotorState& motor_state = joint_state_[i];
        const double motor_limit = joint_torque_limit_[i];

        const double actual_pos = motor.getPosition();
//        const double actual_vel = (actual_pos - motor_state.pos) / period.toSec();
        const double actual_vel = motor.getVelocity();

        const double pos_err = motor_cmd.x_desired - actual_pos;
        const double vel_err = motor_cmd.v_desired - actual_vel;
        double cmd_torque =
                clamp(pos_err * motor_cmd.kp
                      + vel_err * motor_cmd.kd
                      + motor_cmd.torque,
                      - motor_limit,
                      motor_limit);

        if (!mimicking_joints_[i].empty())
        {
            // mimic joints share the total amount of torque
            // they are kept in the same position by mimic joint pd control

            for (hardware_interface::JointHandle& mimicking
                 : mimicking_joints_[i])
            {
                const double mimic_pos_err
                        = actual_pos - mimicking.getPosition();
                const double mimic_vel_err
                        = actual_vel - mimicking.getVelocity();
                const double mimic_torque
                        = mimic_pos_err * mimic_kp_
                        + mimic_vel_err * mimic_kd_;

                mimicking.setCommand(mimic_torque);
                cmd_torque -= mimic_torque;
            }
        }

        motor_state.pos = actual_pos;
        motor_state.vel = actual_vel;
        motor_state.torq = motor.getEffort();

        motor.setCommand(cmd_torque);
    }
}

void GazeboPVEController::commandCB(
        const std_msgs::Float64MultiArrayConstPtr &msg)
{
    if (msg->data.size() != joint_cnt_ * 5)
    {
        ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() <<
                         ") does not match number of joints (" << joint_cnt_
                         << ") * 5! Not executing!");
        return;
    }

    std_msgs::Float64MultiArray state_msg;
    state_msg.data.resize(joint_cnt_ * 3);
    const double* input_msg = msg->data.data();
    double* output_msg = state_msg.data.data();

    for (unsigned int i = 0; i < joint_cnt_; i++)
    {
        MotorCommand& motor_cmd = joint_cmd_[i];
        const MotorState& motor_state = joint_state_[i];

        motor_cmd.x_desired = input_msg[joint_cnt_ * 0 + i];
        motor_cmd.kp        = input_msg[joint_cnt_ * 1 + i];
        motor_cmd.v_desired = input_msg[joint_cnt_ * 2 + i];
        motor_cmd.kd        = input_msg[joint_cnt_ * 3 + i];
        motor_cmd.torque    = input_msg[joint_cnt_ * 4 + i];

        output_msg[joint_cnt_ * 0 + i] = motor_state.pos;
        output_msg[joint_cnt_ * 1 + i] = motor_state.vel;
        output_msg[joint_cnt_ * 2 + i] = motor_state.torq;
    }

    if (publish_state_)
        motor_state_pub_.publish(state_msg);
}

} // namespace effort_controllers

PLUGINLIB_EXPORT_CLASS(effort_controllers::GazeboPVEController,
                       controller_interface::ControllerBase);
