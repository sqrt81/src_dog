#ifndef DOG_CONTROL_UTILS_QUEUEIMPL_HPP
#define DOG_CONTROL_UTILS_QUEUEIMPL_HPP

#include "Queue.h"

namespace dog_control
{

namespace utils
{

template<typename T>
inline void Queue<T>::Push(const T &item)
{
    while (static_cast<unsigned>(q_sz_) >= base_.size())
        Expand();

    base_[end_] = item;
    end_ = (end_ + 1) % base_.size();
    q_sz_++;
}

template<typename T>
inline void Queue<T>::Pop()
{
    std::_Destroy(base_.data() + beg_);
    beg_ = (beg_ + 1) % base_.size();
    q_sz_--;
}

template<typename T>
inline T& Queue<T>::operator[] (int id)
{
    return base_[(beg_ + id) % base_.size()];
}

template<typename T>
inline const T& Queue<T>::operator[] (int id) const
{
    return base_[(beg_ + id) % base_.size()];
}

template<typename T>
inline int Queue<T>::size() const
{
    return q_sz_;
}

template<typename T>
inline void Queue<T>::Expand()
{
    std::vector<T> temp(base_.size() * 2);

    for (int i = 0; i < q_sz_; i++)
    {
        temp[i] = std::move(base_[(beg_ + i) % base_.size()]);
    }

    beg_ = 0;
    end_ = q_sz_;

    base_.swap(temp);
}

} /* utils */

} /* dog_control */

#endif /* DOG_CONTROL_UTILS_QUEUEIMPL_HPP */
