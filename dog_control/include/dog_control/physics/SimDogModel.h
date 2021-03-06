#ifndef DOG_CONTROL_PHYSICS_SIMDOGMODEL_H
#define DOG_CONTROL_PHYSICS_SIMDOGMODEL_H

#include "dog_control/physics/DogModel.h"

namespace dog_control
{

namespace physics
{

class SimDogModel : protected DogModel
{
public:
    using DogModel::Initialize;

    void Update(FBJSCRef stat);

    using spatial::FloatingBaseModel::ToggleDiff;
//    using spatial::FloatingBaseModel::SetJointMotionState;
//    using spatial::FloatingBaseModel::ForwardKinematics;
    using spatial::FloatingBaseModel::MassMatrix;
    using spatial::FloatingBaseModel::MassMatrixDiff;
    using spatial::FloatingBaseModel::BiasForces;
    using spatial::FloatingBaseModel::BiasForceDiff;
    using spatial::FloatingBaseModel::BaseForceJacobian;

//    using DogModel::FullJacobMat;
    using DogModel::FullJacob;
    using DogModel::FullJacobBaseDiff;
    using DogModel::FullJacobJointDiff;
};

} /* physics */

} /* dog_control */

#endif /* DOG_CONTROL_PHYSICS_SIMDOGMODEL_H */
