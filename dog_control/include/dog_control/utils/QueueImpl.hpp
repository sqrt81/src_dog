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

    std::_Construct(M_storage_ + end_, item);
//    M_storage_[end_] = item;
    end_ = (end_ + 1) % base_.size();
    q_sz_++;
}

template<typename T>
inline void Queue<T>::Push(T &&item)
{
    while (static_cast<unsigned>(q_sz_ + 1) >= base_.size())
        Expand();

    std::_Construct(M_storage_ + end_, item);
//    M_storage_[end_] = item;
    end_ = (end_ + 1) % base_.size();
    q_sz_++;
}

template<typename T>
inline void Queue<T>::Pop()
{
    std::_Destroy(M_storage_ + beg_);
    beg_ = (beg_ + 1) % base_.size();
    q_sz_--;
}

template<typename T>
inline void Queue<T>::Clear()
{
    if (beg_ < end_)
    {
        std::_Destroy(M_storage_ + beg_, M_storage_ + end_);
    }
    else
    {
        std::_Destroy(M_storage_ + beg_, M_storage_ + base_.size());
        std::_Destroy(M_storage_, M_storage_ + end_);
    }

    beg_ = 0;
    end_ = 0;
    q_sz_ = 0;
}

template<typename T>
inline void Queue<T>::EraseTail(int n_erase)
{
    if (n_erase <= 0)
        return;

    if (n_erase >= q_sz_)
    {
        Clear();
        return;
    }

    q_sz_ -= n_erase;

    if (end_ >= n_erase)
    {
        std::_Destroy(M_storage_ + end_ - n_erase, M_storage_ + end_);
        end_ -= n_erase;
    }
    else // beg_ > end_ && end_ < n_erase
    {
        std::_Destroy(M_storage_ + base_.size() + end_ - n_erase,
                      M_storage_ + base_.size());
        std::_Destroy(M_storage_, M_storage_ + end_);

        end_ = beg_ + q_sz_;
    }
}

template<typename T>
inline void Queue<T>::EraseHead(int n_erase)
{
    if (n_erase <= 0)
        return;

    if (n_erase >= q_sz_)
    {
        Clear();
        return;
    }

    q_sz_ -= n_erase;

    if (static_cast<signed>(base_.size()) - beg_ > n_erase)
    {
        std::_Destroy(M_storage_ + beg_, M_storage_ + beg_ + n_erase);
        beg_ += n_erase;
    }
    else // beg_ > end_ && base_.size() - beg_ < n_erase
    {
        std::_Destroy(M_storage_ + beg_, M_storage_ + base_.size());
        std::_Destroy(M_storage_,
                      M_storage_ - base_.size() + beg_ + n_erase);

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
    return M_storage_[(beg_ + id) % base_.size()];
}

template<typename T>
inline const T& Queue<T>::operator[] (int id) const
{
    return M_storage_[(beg_ + id) % base_.size()];
}

template<typename T>
inline T& Queue<T>::Head()
{
    return M_storage_[beg_];
}

template<typename T>
inline const T& Queue<T>::Head() const
{
    return M_storage_[beg_];
}

template<typename T>
inline T& Queue<T>::Tail()
{
    return M_storage_[(end_ - 1) % base_.size()];
}

template<typename T>
inline const T& Queue<T>::Tail() const
{
    return M_storage_[(end_ - 1) % base_.size()];
}

template<typename T>
inline void Queue<T>::Expand()
{
    std::vector<T_storage> temp(base_.size() * 2);
    T* const M_new = reinterpret_cast<T*>(temp.data());

    for (int i = 0; i < q_sz_; i++)
    {
        std::_Construct(M_new + i,
                        std::move(M_storage_[(beg_ + i) % base_.size()]));
//        M_new[i] = std::move(M_storage_[(beg_ + i) % base_.size()]);
    }

    beg_ = 0;
    end_ = q_sz_;

    base_.swap(temp);
    M_storage_ = reinterpret_cast<T*>(base_.data());
}

} /* utils */

} /* dog_control */

#endif /* DOG_CONTROL_UTILS_QUEUEIMPL_HPP */
