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
    while (static_cast<unsigned>(q_sz_ + 1) >= base_.size())
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
inline void Queue<T>::Clear()
{
    if (beg_ < end_)
    {
        std::_Destroy(base_.data() + beg_, base_.data() + end_);
    }
    else
    {
        std::_Destroy(base_.data() + beg_, base_.data() + base_.size());
        std::_Destroy(base_.data(), base_.data() + end_);
    }

    beg_ = 0;
    end_ = 0;
    q_sz_ = 0;
}

template<typename T>
inline void Queue<T>::EraseTail(int n_erase)
{
    if (n_erase >= q_sz_)
    {
        Clear();
        return;
    }

    q_sz_ -= n_erase;

    if (end_ >= n_erase)
    {
        std::_Destroy(base_.data() + end_ - n_erase, base_.data() + end_);
        end_ -= n_erase;
    }
    else // beg_ > end_ && end_ < n_erase
    {
        std::_Destroy(base_.data() + base_.size() + end_ - n_erase,
                      base_.data() + base_.size());
        std::_Destroy(base_.data(), base_.data() + end_);

        end_ = beg_ + q_sz_;
    }
}

template<typename T>
inline void Queue<T>::EraseHead(int n_erase)
{
    if (n_erase >= q_sz_)
    {
        Clear();
        return;
    }

    q_sz_ -= n_erase;

    if (base_.size() - beg_ >= n_erase)
    {
        std::_Destroy(base_.data() + beg_, base_.data() + beg_ + n_erase);
        beg_ += n_erase;
    }
    else // beg_ > end_ && base_.size() - beg_ < n_erase
    {
        std::_Destroy(base_.data() + beg_, base_.data() + base_.size());
        std::_Destroy(base_.data(),
                      base_.data() - base_.size() + beg_ + n_erase);

        beg_ = end_ - q_sz_;
    }
}

template<typename T>
inline int Queue<T>::size() const
{
    return q_sz_;
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
inline T& Queue<T>::Head()
{
    return base_[beg_];
}

template<typename T>
inline const T& Queue<T>::Head() const
{
    return base_[beg_];
}

template<typename T>
inline T& Queue<T>::Tail()
{
    return base_[(end_ - 1) & base_.size()];
}

template<typename T>
inline const T& Queue<T>::Tail() const
{
    return base_[(end_ - 1) & base_.size()];
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
