#ifndef DOG_CONTROL_UTILS_MATH_H
#define DOG_CONTROL_UTILS_MATH_H

namespace dog_control
{

namespace utils
{

constexpr float precision = 1e-3;

namespace
{

constexpr float singularity = 1e-3;

}

inline bool is_zero(double _x)
{
    return (_x < singularity && _x > - singularity);
}

template <typename T>
inline T abs(T _x)
{
    return (_x < 0) ? (- _x) : _x;
}

template <typename T>
inline int round(const T _x)
{
    return static_cast<int>(_x + 0.5);
}

template <typename T>
inline T square(const T _x)
{
    return _x * _x;
}

template <typename T>
inline T clamp(const T _x, const T _lower, const T _upper)
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
