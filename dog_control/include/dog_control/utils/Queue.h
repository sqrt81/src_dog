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
private:
    union T_storage
    {
        T item;
        char data[sizeof(T)];

        T_storage() {}
        ~T_storage() {}
    };

public:
    Queue()
     : base_(1), M_storage_(reinterpret_cast<T*>(base_.data())),
       beg_(0), end_(0), q_sz_(0)
    {}

    void Push(const T &item);

    void Push(T&& item);

    void Pop();

    void Clear();

    void EraseTail(int n_erase);

    void EraseHead(int n_erase);

    int size() const;

    T& operator[] (int id);

    const T& operator[] (int id) const;

    T& Head();

    const T& Head() const;

    T& Tail();

    const T& Tail() const;

private:
    void Expand();

    std::vector<T_storage> base_;
    T* M_storage_;

    int beg_;
    int end_;
    int q_sz_;
};

} /* utils */

} /* dog_control */

#endif /* DOG_CONTROL_UTILS_QUEUE_H */
