#ifndef DOG_CONTROL_ESTIMATOR_ESTIMATORBASE_H
#define DOG_CONTROL_ESTIMATOR_ESTIMATORBASE_H

#include "dog_control/utils/ClassDeclare.h"

#include "dog_control/message/EstimatorResult.h"
#include "dog_control/utils/ParamDict.h"

#include <boost/weak_ptr.hpp>

namespace dog_control
{

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

    EstimatorResult WriteResult() const
    {
        return res_;
    }

protected:
    boost::weak_ptr<hardware::HardwareBase> hw_ptr_;
    boost::weak_ptr<physics::DogModel> model_ptr_;

    EstimatorResult res_;
};

} /* estimator */

} /* dog_control */

#endif /* DOG_CONTROL_ESTIMATOR_ESTIMATORBASE_H */
