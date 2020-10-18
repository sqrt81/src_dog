#ifndef DOG_CONTROL_UTILS_CUBICSPLINE_H
#define DOG_CONTROL_UTILS_CUBICSPLINE_H

namespace dog_control
{

namespace utils
{

/**
 * @brief The CubicSpline class
 * Used for cubic interpolation.
 * This class generate a cubic polynomial function f(x) that satisfies:
 * f(t1) = x1;  f'(t1) = v1;
 * f(t2) = x2;  f'(t2) = v2;
 */
template <typename T, typename dT = T>
class CubicSpline
{
public:
    CubicSpline() = default;

    CubicSpline(double t1, const T &x1, const dT &v1,
                double t2, const T &x2, const dT &v2);

    /**
     * @brief Sample
     * Obtain the value of the function and its derivative
     * at time t.
     * Here, x = f(t), v = f'(t)
     */
    void Sample(double t, T &x, dT &v) const;

    /**
     * @brief Sample
     * Obtain the value of the function and its
     * first and second order derivative at time t.
     * x = f(t), v = f'(t), a = f''(t)
     */
    void Sample(double t, T &x, dT &v, dT& a) const;

private:
    double t0_;
    T a0_;
    dT a1_;
    dT a2_;
    dT a3_;
};

} /* utils */

} /* dog_control */

#endif /* DOG_CONTROL_UTILS_CUBICSPLINE_H */
