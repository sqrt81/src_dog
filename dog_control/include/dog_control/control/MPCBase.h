#ifndef DOG_CONTROL_CONTROL_MPCBASE_H
#define DOG_CONTROL_CONTROL_MPCBASE_H

#include "dog_control/utils/ClassDeclare.h"

#include "dog_control/message/ModelJointState.h"
#include "dog_control/utils/ParamDict.h"

#include <Eigen/Eigen>
#include <array>
#include <vector>
#include <boost/weak_ptr.hpp>

namespace dog_control
{

namespace control
{

class MPCBase
{
protected:
    using FBState = message::FloatingBaseState;
    using FBStateAcc = message::FloatingBaseStateWithAcc;
    using FBSCRef = message::FBStateCRef;

public:
    using TorsoTraj = std::vector<FBState>;
    using FeetPosSeq = std::vector<std::array<Eigen::Vector3d, 4>>;
    using FeetContactSeq = std::vector<std::array<bool, 4>>;

    MPCBase() = default;

    virtual void Initialize(utils::ParamDictCRef dict) = 0;

    void ConnectClock(boost::shared_ptr<hardware::ClockBase> clock)
    {
        clock_ptr_ = clock;
    }

    void ConnectTraj(boost::shared_ptr<control::TrajectoryController> traj)
    {
        traj_ptr_ = traj;
    }

    void ConnectModel(boost::shared_ptr<physics::DogModel> model)
    {
        model_ptr_ = model;
    }

    virtual FBStateAcc GetTorsoState(double t) = 0;

    virtual void GetFeetForce(
            double t,
            std::array<Eigen::Vector3d, 4> &force,
            std::array<bool, 4> &contact) const = 0;

    virtual void Update() = 0;

protected:
    Eigen::VectorXd state_weight_;
    double force_weight_;

    double ground_friction_;
    double f_z_max_;
    double f_z_min_;    // minimal z force to keep foot in contact
    Eigen::Vector3d gravity_;

    // update_period_ is MPC update period,
    // i.e., the time interval between two predictions.
    double update_period_;

    // Parameter
    double last_update_time_;       // time stamp of last prediction
    double update_dt_;              // basic control period

    boost::weak_ptr<control::TrajectoryController> traj_ptr_;
    boost::weak_ptr<hardware::ClockBase> clock_ptr_;
    boost::weak_ptr<physics::DogModel> model_ptr_;

    FBState cur_state_;
    TorsoTraj desired_traj_;
    FeetPosSeq feet_pos_seq_;
    FeetContactSeq contact_seq_;

    // result storage
    std::vector<std::array<Eigen::Vector3d, 4>> foot_force_;
};

} /* control */

} /* dog_control */

#endif // MPCBASE_H
