#include "dog_control/utils/Queue.h"
#include "dog_control/utils/QueueImpl.hpp"
#include "dog_control/utils/MiniLog.h"

using namespace dog_control;

int test_Queue()
{
    utils::Queue<double> q;

    for(int i = 0; i < 10; i++)
    {
        q.Push(i);

        LOG(INFO) << "tail: " << q[q.size() - 1] << " size: " << q.size();
    }

    for(int i = 0; i < 5; i++)
    {
        LOG(INFO) << "head: " << q[0];
        q.Pop();
    }

    for(int i = 0; i < 10; i++)
    {
        q.Push(i + 10);

        LOG(INFO) << "tail: " << q[q.size() - 1] << " size: " << q.size();
    }

    LOG(INFO) << "random access: " << q[12];
    LOG(INFO) << "random access: " << q[5 ];
    LOG(INFO) << "random access: " << q[3 ];

    for(int i = 0; i < 15; i++)
    {
        LOG(INFO) << "head: " << q[0];
        q.Pop();
    }

    return 0;
}
