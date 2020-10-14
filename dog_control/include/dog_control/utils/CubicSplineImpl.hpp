#include "dog_control/utils/CubicSpline.h"

#include "dog_control/utils/MiniLog.h"

namespace dog_control
{

namespace utils
{

template <typename T>
CubicSpline<T>::CubicSpline(double t1, T x1, T v1,
                         double t2, T x2, T v2)
{
    CHECK(t1 != t2);

    t0_ = t1;
    a0_ = x1;
    a1_ = v1;
    double inv_delta_t = 1. / (t2 - t1);
    double par1 = (x2 - x1) * inv_delta_t - v1;
    double par2 = v2 - v1;
    a2_ = (par1 * 3 - par2) * inv_delta_t;
    a3_ = (par1 * inv_delta_t - a2_) * inv_delta_t;
}

template <typename T>
void CubicSpline<T>::Sample(double t, T& x, T& v) const
{
    double delta_t = t - t0_;
    double delta_t_2 = delta_t * delta_t;

    x = a3_ * delta_t * delta_t_2 + a2_ * delta_t_2 + a1_ * delta_t + a0_;
    v = 3 * a3_ * delta_t_2 + 2 * a2_ * delta_t + a1_;
}

template <typename T>
void CubicSpline<T>::Sample(double t, T &x, T &v, T& a) const
{
    double delta_t = t - t0_;
    double delta_t_2 = delta_t * delta_t;

    x = a3_ * delta_t * delta_t_2 + a2_ * delta_t_2 + a1_ * delta_t + a0_;
    v = 3 * a3_ * delta_t_2 + 2 * a2_ * delta_t + a1_;
    a = 6 * a3_ * delta_t + 2 * a2_;
}

} /* utils */

} /* dog_control */

