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

    In modules, it is common to leave interfaces for inputing or
    outputing data. However, in order to keep the logic clear,
    we here suggest every module to use only "query" interfaces.
    That is to say, in Update() method, a module may obtain data
    from other modules by calling upon corresponding methods,
    but it may not set any data for other modules.
    This rule will lead to a result that a module only connects
    to the modules it rely upon, but not to those rely on it.
    In this way, the relationship of modules are made clear,
    and the order of update can be decided.
*/

#ifndef DOG_CONTROL_UTILS_CLASSDECLARE_H
#define DOG_CONTROL_UTILS_CLASSDECLARE_H

namespace dog_control
{

namespace control
{

class FootPosController;
class MPCBase;
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
class SimDogModel;

} /* physics */

} /* dog_control */

#endif /* DOG_CONTROL_UTILS_CLASSDECLARE_H */
