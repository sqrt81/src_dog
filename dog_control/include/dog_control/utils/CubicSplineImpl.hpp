#ifndef DOG_CONTROL_UTILS_CUBICSPLINIMPL_HPP
#define DOG_CONTROL_UTILS_CUBICSPLINIMPL_HPP

#include "dog_control/utils/CubicSpline.h"

#include "dog_control/utils/MiniLog.h"

namespace dog_control
{

namespace utils
{

template <typename T, typename dT>
CubicSpline<T, dT>::CubicSpline(double t1, const T &x1, const dT &v1,
                                double t2, const T &x2, const dT &v2)
{
    static_assert (std::is_same<T, dT>::value,
            "[CubicSpline] Value and derivate type should be same.");

    CHECK(t1 != t2) << "[CubicSpline] Invalid mark points.";

    t0_ = t1;
    a0_ = x1;
    a1_ = v1;
    const double inv_delta_t = 1. / (t2 - t1);
    T par1 = (x2 - x1) * inv_delta_t - v1;
    T par2 = v2 - v1;
    a2_ = (par1 * 3 - par2) * inv_delta_t;
    a3_ = (par1 * inv_delta_t - a2_) * inv_delta_t;
}

template <typename T, typename dT>
void CubicSpline<T, dT>::Sample(double t, T& x, dT& v) const
{
    double delta_t = t - t0_;
    double delta_t_2 = delta_t * delta_t;

    x = (a3_ * delta_t + a2_) * delta_t_2 + a1_ * delta_t + a0_;
    v = 3 * a3_ * delta_t_2 + 2 * a2_ * delta_t + a1_;
}

template <typename T, typename dT>
void CubicSpline<T, dT>::Sample(double t, T &x, dT &v, dT &a) const
{
    double delta_t = t - t0_;
    double delta_t_2 = delta_t * delta_t;

    x = (a3_ * delta_t + a2_) * delta_t_2 + a1_ * delta_t + a0_;
    v = 3 * a3_ * delta_t_2 + 2 * a2_ * delta_t + a1_;
    a = 6 * a3_ * delta_t + 2 * a2_;
}

} /* utils */

} /* dog_control */

#endif /* DOG_CONTROL_UTILS_CUBICSPLINIMPL_HPP */
