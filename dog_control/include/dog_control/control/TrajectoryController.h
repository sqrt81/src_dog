#ifndef DOG_CONTROL_CONTROL_TRAJECTORYCONTROLLER_H
#define DOG_CONTROL_CONTROL_TRAJECTORYCONTROLLER_H

#include "dog_control/utils/ClassDeclare.h"

#include "dog_control/message/ModelJointState.h"
#include "dog_control/utils/ParamDict.h"
#include "dog_control/utils/Queue.h"

#include <boost/weak_ptr.hpp>

namespace dog_control
{

namespace control
{

class TrajectoryController
{
protected:
    using FBState = message::FloatingBaseStateWithAcc;
    using StampedFBS = message::StampedFloatingBaseState;

public:
    using TorsoTraj = std::vector<StampedFBS>;

    TrajectoryController() = default;

    void Initialize(utils::ParamDictCRef dict);

    void ConnectClock(boost::shared_ptr<hardware::ClockBase> clock);

    void SetTorsoTrajectory(const TorsoTraj &torso_traj);

    FBState GetTorsoState(double t) const;

    void GetTorsoStates(const std::vector<double>& time_stamp,
                       std::vector<FBState>& poses);

    void Update();
private:
    double dt_;
    int traj_record_len_;

    double traj_beg_time_;

    boost::weak_ptr<hardware::ClockBase> clock_ptr_;

    utils::Queue<FBState> torso_traj_;

};

} /* control */

} /* dog_control */

#endif /* DOG_CONTROL_CONTROL_TRAJECTORYCONTROLLER_H */
