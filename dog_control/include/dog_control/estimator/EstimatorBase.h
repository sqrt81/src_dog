#ifndef DOG_CONTROL_ESTIMATOR_ESTIMATORBASE_H
#define DOG_CONTROL_ESTIMATOR_ESTIMATORBASE_H

#include "dog_control/hardware/HardwareBase.h"
#include "dog_control/message/EstimatorResult.h"

#include <boost/weak_ptr.hpp>

namespace dog_control
{

namespace physics
{

class DogModel;

}

namespace estimator
{

class EstimatorBase
{
protected:
    using EstimatorResult = message::EstimatorResult;
public:
    EstimatorBase() = default;

    virtual void Initialize(utils::ParamDictCRef dict) = 0;

    void ConnectHardware(boost::shared_ptr<hardware::HardwareBase> hw)
    {
        hw_ptr_ = hw;
    }

    void ConnectModel(boost::shared_ptr<physics::DogModel> model)
    {
        model_ptr_ = model;
    }

    virtual void Update() = 0;

    virtual void WriteResult(EstimatorResult& result) const = 0;

protected:
    boost::weak_ptr<hardware::HardwareBase> hw_ptr_;
    boost::weak_ptr<physics::DogModel> model_ptr_;

    EstimatorResult res_;
};

} /* estimator */

} /* dog_control */

#endif /* DOG_CONTROL_ESTIMATOR_ESTIMATORBASE_H */
