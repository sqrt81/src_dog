#ifndef DOG_CONTROL_UTILS_MATH_H
#define DOG_CONTROL_UTILS_MATH_H

namespace dog_control
{

namespace utils
{

namespace
{

constexpr float singularity = 1e-3;

}

inline bool is_zero(double _x)
{
    return (_x < singularity && _x > - singularity);
}

inline double abs(double _x)
{
    return (_x < 0) ? (- _x) : _x;
}

inline int round(const double _x)
{
    return static_cast<int>(_x + 0.5);
}

inline double square(const double _x)
{
    return _x * _x;
}

inline double clamp(const double _x, const double _lower, const double _upper)
{
    if (_x > _upper)
        return _upper;

    if (_x < _lower)
        return _lower;

    return _x;
}

} /* utils */

} /* dog_control */

#endif /* DOG_CONTROL_UTILS_MATH_H */
