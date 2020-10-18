#ifndef DOG_CONTROL_UTILS_QUATSPLINEIMPL_HPP
#define DOG_CONTROL_UTILS_QUATSPLINEIMPL_HPP

#include "dog_control/utils/CubicSpline.h"

#include "dog_control/utils/Math.h"
#include "dog_control/utils/MiniLog.h"

#include <Eigen/Eigen>

namespace dog_control
{

namespace utils
{

template <>
CubicSpline<Eigen::Quaterniond, Eigen::Vector3d>::CubicSpline(
        double t1, const Eigen::Quaterniond &x1, const Eigen::Vector3d &v1,
        double t2, const Eigen::Quaterniond &x2, const Eigen::Vector3d &v2)
{
    CHECK(t1 != t2) << "[CubicSpline] Invalid mark points.";

    t0_ = t1;
    a0_ = x1;
    a1_ = v1;
    const double delta_t = t2 - t1;
    const double inv_delta_t = 1. / delta_t;
    const double inv_delta_t_2 = square(inv_delta_t);

    const Eigen::Quaterniond delta_q = x1.conjugate() * x2;
    // vf is rotational vel v2 expressed in x1's frame
    const Eigen::Vector3d vf = v2; //delta_q.conjugate() * v2;

    // norm is the rotation angle, and vec is the unit rotation direction.
    const double norm = 2 * std::atan2(delta_q.vec().norm(), delta_q.w());
    Eigen::Vector3d theta;
    Eigen::Vector3d dtheta;

    if (is_zero(norm))
    {
        theta = Eigen::Vector3d::Zero();
        dtheta = vf;
    }
    else
    {
        const Eigen::Vector3d vec = delta_q.vec().normalized();
        const double coef = 0.5 * (norm * sin(norm) / (1 - cos(norm)));

        theta = norm * vec;
        dtheta = (1 - coef) * vec.dot(vf) * vec
               + coef * vf
               + 0.5 * norm * vec.cross(vf);
    }

    theta -= delta_t * v1;
    dtheta -= v1;

    a3_ = (dtheta - 2 * theta * inv_delta_t) * inv_delta_t_2;
    a2_ = theta * inv_delta_t_2 - a3_ * delta_t;
}

template <>
void CubicSpline<Eigen::Quaterniond, Eigen::Vector3d>::Sample(
        double t, Eigen::Quaterniond &x, Eigen::Vector3d &v) const
{
    double delta_t = t - t0_;
    double delta_t_2 = delta_t * delta_t;

    const Eigen::Vector3d theta
            = (a3_ * delta_t_2 + a2_ * delta_t + a1_) * delta_t;
    const Eigen::Vector3d dtheta
            = 3 * a3_ * delta_t_2 + 2 * a2_ * delta_t + a1_;

    const double norm = theta.norm();

    if (is_zero(norm))
    {
        // dq = Identity()
        x = a0_;
        v = dtheta;
    }
    else
    {
        const Eigen::Vector3d vec = theta.normalized();
        const Eigen::Quaterniond dq(
                    Eigen::AngleAxisd(norm, theta.normalized()));
        const double sth = sin(norm);
        const double cth_1 = 1 - cos(norm);

        x = a0_ * dq;
        v = /*dq.conjugate() * */(
                    (norm - sth) * vec.dot(dtheta) * vec
                    + sth * dtheta
                    - cth_1 * vec.cross(dtheta)) / norm;
    }
}

template <>
void CubicSpline<Eigen::Quaterniond, Eigen::Vector3d>::Sample(
        double t,
        Eigen::Quaterniond &x,
        Eigen::Vector3d &v,
        Eigen::Vector3d &a) const
{
    double delta_t = t - t0_;
    double delta_t_2 = delta_t * delta_t;

    const Eigen::Vector3d theta
            = (a3_ * delta_t_2 + a2_ * delta_t + a1_) * delta_t;
    const Eigen::Vector3d dtheta
            = 3 * a3_ * delta_t_2 + 2 * a2_ * delta_t + a1_;
    const Eigen::Vector3d ddtheta
            = 6 * a3_ * delta_t + 2 * a2_;

    const double norm = theta.norm();

    if (is_zero(norm))
    {
        // dq = Identity()
        x = a0_;
        v = dtheta;
        a = ddtheta;
    }
    else
    {
        const Eigen::Vector3d vec = theta.normalized();
        const Eigen::Quaterniond dq(
                    Eigen::AngleAxisd(norm, theta.normalized()));
        // w = d(vec) / dt;
        const Eigen::Vector3d w = vec.cross(dtheta) / norm;
        const Eigen::Vector3d dw
                = (vec.cross(ddtheta) - 2 * vec.dot(dtheta) * w) / norm;
        const double dnorm = vec.dot(dtheta);
        const double ddnorm = w.cross(vec).dot(dtheta) + vec.dot(ddtheta);
        const double sth = sin(norm);
        const double cth_1 = 1 - cos(norm);

        x = a0_ * dq;
        v = ((norm - sth) * vec.dot(dtheta) * vec
             + sth * dtheta
             - cth_1 * vec.cross(dtheta)) / norm;
        a = /*dq.conjugate() * */(
                    ddnorm * vec
                    + sth * dw.cross(vec)
                    - cth_1 * dw
                    + dnorm * w.cross(vec)
                    + v.cross(dnorm * vec - w));
//        v = dq.conjugate() * v;
    }
}

} /* utils */

} /* dog_control */

#endif /* DOG_CONTROL_UTILS_QUATSPLINEIMPL_HPP */
