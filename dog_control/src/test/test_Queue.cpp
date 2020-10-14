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

    q.EraseTail(7);

    LOG(INFO) << "random access: " << q[5 ];
    LOG(INFO) << "random access: " << q[3 ];

    LOG(INFO) << q.size();

    q.Clear();

    LOG(INFO) << q.size();

    return 0;
}
