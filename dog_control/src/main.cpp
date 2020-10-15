#include "dog_control/utils/Queue.h"
#include "dog_control/utils/QueueImpl.hpp"
#include "dog_control/utils/MiniLog.h"

using namespace dog_control;

static int cnt = 0;

class test_class
{
public:
    int this_class;

    int kkk;

    test_class()
    {
        this_class = cnt;
        kkk = 1;
        cnt++;

        LOG(DEBUG) << "create entity " << this_class;
    }

    test_class(const test_class& another)
    {
        LOG(DEBUG) << "lvalue called";
        (void) another;
        std::_Construct_novalue(this);
    }

    test_class(test_class&& another)
    {
        LOG(DEBUG) << "rvalue called";
        another.kkk = 0;
        this_class = another.this_class;
        kkk = 1;

        LOG(DEBUG) << "change control of entity " << this_class;
    }

    ~test_class()
    {
        if (kkk == 1)
        {
//            cnt--;

            LOG(DEBUG) << "destroy entity " << this_class;
        }
    }
};

int main()
{
    utils::Queue<test_class> q;

    for(int i = 0; i < 20; i++)
    {
        q.Push(test_class());

        LOG(INFO) << "tail: " << q[q.size() - 1].this_class
                  << " size: " << q.size();
    }

    for(int i = 0; i < 5; i++)
    {
        LOG(INFO) << "head: " << q[0].this_class;
        q.Pop();
    }

    q.EraseTail(7);

    LOG(INFO) << "random access: " << q[5 ].this_class;
    LOG(INFO) << "random access: " << q[3 ].this_class;

    LOG(INFO) << q.size();

    q.Clear();

    LOG(INFO) << q.size();

    return 0;
}
