#ifndef DOG_ROS_VISUALIZATION_RVIZVISUALIZATION_H
#define DOG_ROS_VISUALIZATION_RVIZVISUALIZATION_H

#include "dog_control/message/VisualData.h"

#include <ros/ros.h>

namespace dog_control
{

namespace visualization
{

class RvizVisualization
{
protected:
    using VisualData = message::VisualData;

public:
    RvizVisualization();

    void SetData(const VisualData& data);

    void Update();

private:
    ros::NodeHandle nh_;
    ros::Publisher est_pos_pub_;
    ros::Publisher torso_traj_pub_;
    ros::Publisher foot_traj_pub_[4];
    ros::Publisher foot_force_pub_;

    const VisualData* data_;

};

} /* visualization */

} /* dog_control */

#endif /* DOG_ROS_VISUALIZATION_RVIZVISUALIZATION_H */
