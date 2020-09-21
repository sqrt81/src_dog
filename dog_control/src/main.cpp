#include "dog_control/utils/Initializer.h"
#include "dog_control/physics/DogPhysics.h"

#include <ros/ros.h>
#include <iostream>

using namespace std;
using namespace dog_control;

int main(int argc, char** argv)
{
    utils::ParamDict dict = utils::BuildParamDict(
                "/home/sqrt81/catkin_ws/src_dog/test.lua");
    physics::DogPhysics phys;
    phys.Initialize(dict);

    for(auto item : dict)
        cout << "key: " << item.first << " value: " << item.second << endl;

    cout << "output success!" << endl;

    return 0;
}
