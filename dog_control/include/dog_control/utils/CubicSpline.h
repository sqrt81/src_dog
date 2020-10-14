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
class CubicSpline
{
public:
    CubicSpline();

    CubicSpline(double t1, double x1, double v1,
                double t2, double x2, double v2);

    /**
     * @brief Sample
     * Obtain the value of the function and its derivative
     * at time t.
     * Here, x = f(t), v = f'(t)
     */
    void Sample(double t, double &x, double &v) const;

    /**
     * @brief Sample
     * Obtain the value of the function and its
     * first and second order derivative at time t.
     * x = f(t), v = f'(t), a = f''(t)
     */
    void Sample(double t, double &x, double &v, double& a) const;

private:
    double t0_;
    double a0_;
    double a1_;
    double a2_;
    double a3_;
}; /* CubicSpline */

} /* utils */

} /* dog_control */

#endif /* DOG_CONTROL_UTILS_CUBICSPLINE_H */
