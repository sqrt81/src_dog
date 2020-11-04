#ifndef DOG_CONTROL_CONTROL_TRAJECTORYCONTROLLER_H
#define DOG_CONTROL_CONTROL_TRAJECTORYCONTROLLER_H

#include "dog_control/utils/ClassDeclare.h"

#include "dog_control/control/FootSwingTrajBase.h"
#include "dog_control/message/ModelJointState.h"
#include "dog_control/message/LegName.h"
#include "dog_control/utils/ParamDict.h"
#include "dog_control/utils/Queue.h"

#include <boost/weak_ptr.hpp>

namespace dog_control
{

namespace control
{

/**
 * @brief The TrajectoryController class
 * Trajectory controller is not really a controller, for it
 * controls nothing. Instead, it generates torso and foot trajectory
 * and allow low-level controllers to query torso & foot locations.
 * Thus, trajectory controller serves as an encapsulation of
 * low-level controllers. High-level planners only needs to give
 * waypoints of torso and feet, and leave the controlling behind.
 */
class TrajectoryController
{
protected:
    using MPCState = message::FloatingBaseState;
    using FBState = message::FloatingBaseStateWithAcc;
    using StampedFBS = message::StampedFloatingBaseState;

public:
    using TorsoTraj = std::vector<StampedFBS>;

    TrajectoryController() = default;

    void Initialize(utils::ParamDictCRef dict);

    void ConnectClock(boost::shared_ptr<hardware::ClockBase> clock);
    void ConnectModel(boost::shared_ptr<physics::DogModel> model);

    void UpdateFootPos();

    void SetTorsoTrajectory(const TorsoTraj &torso_traj);

    void ClearTorsoTrajectory();

    void SetFootTrajectory(message::LegName foot_name,
                           boost::shared_ptr<FootSwingTrajBase> traj);

    FBState GetTorsoState(double t) const;

    void GetCurLocalFootState(message::LegName foot_name,
                              Eigen::Vector3d &pos,
                              Eigen::Vector3d &vel,
                              bool &hip_out,
                              bool &knee_out) const;

    void GetFootState(double t, message::LegName foot_name,
                      Eigen::Vector3d &pos,
                      Eigen::Vector3d &vel,
                      Eigen::Vector3d &acc,
                      bool &contact) const;

    void GetTorsoTraj(const std::vector<double>& time_stamp,
                      std::vector<FBState>& traj) const;

    void SampleTrajFromNow(int sample_cnt, double interval,
                           std::vector<MPCState>& traj) const;

    void SampleFootStateFromNow(
            int sample_cnt, double interval,
            std::vector<std::array<Eigen::Vector3d, 4>> &pos_seq,
            std::vector<std::array<bool, 4>> &contact_seq) const;

    void Update();

private:
    int GetIndexForTime(double t) const;

    double dt_;
    int traj_record_len_;

    double traj_beg_time_;

    boost::weak_ptr<hardware::ClockBase> clock_ptr_;
    boost::weak_ptr<physics::DogModel> model_ptr_;

    utils::Queue<FBState> torso_traj_;
    std::array<boost::shared_ptr<FootSwingTrajBase>, 4> swing_traj_;

    // stance leg foot state (global)
    std::array<Eigen::Vector3d, 4> foot_pos_;
};

} /* control */

} /* dog_control */

#endif /* DOG_CONTROL_CONTROL_TRAJECTORYCONTROLLER_H */
