/*
    Declares all "class modules" used in controlling program.
    Since these modules calls each other's methods during
    controlling progress, they must store pointers pointing at
    other module objects.
    So intuitively, in module A's header file, module B will be
    included by #include "module/B.h", if A will call B's method.
    This will lead to a compilation problem when two module
    stores pointer pointing at each other, as the header files
    are recursively included.
    In order to eliminate this problem once and for all, we here
    make the following rule.

        In the header file of a module, it is forbidden to
        include module header files or to make forward declarations
        of other modules. If needed, include this file instead.
*/

#ifndef DOG_CONTROL_UTILS_CLASSDECLARE_H
#define DOG_CONTROL_UTILS_CLASSDECLARE_H

namespace dog_control
{

namespace control
{

class FootPosController;
class ModelPredictiveController;
class TrajectoryController;
class WholeBodyController;

} /* control */

namespace estimator
{

class EstimatorBase;

} /* estimator */

namespace hardware
{

class ClockBase;
class HardwareBase;

} /* hardware */

namespace physics
{

class DogModel;

} /* physics */

} /* dog_control */

#endif /* DOG_CONTROL_UTILS_CLASSDECLARE_H */
