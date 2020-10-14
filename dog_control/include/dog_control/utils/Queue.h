#ifndef DOG_CONTROL_UTILS_QUEUE_H
#define DOG_CONTROL_UTILS_QUEUE_H

#include <vector>

namespace dog_control
{

namespace utils
{

template<typename T>
class Queue
{
public:
    Queue()
     : base_(1), beg_(0), end_(0), q_sz_(0)
    {}

    void Push(const T &item);

    void Pop();

    T& operator[] (int id);

    const T& operator[] (int id) const;

    int size() const;

public:
    void Expand();

    std::vector<T> base_;
    int beg_;
    int end_;
    int q_sz_;
};

} /* utils */

} /* dog_control */

#endif /* DOG_CONTROL_UTILS_QUEUE_H */
