#include "utils.h"

Eigen::Vector2d discreteProjection2d(const Eigen::Vector2d &v, double bound)
{
    return v.norm() > bound ? v.normalized() * bound : v;
}

Eigen::Vector2d discreteProjection2dOut(const Eigen::Vector2d &v, double bound)
{
    return v.norm() < bound ? v.normalized() * bound : v;
}

Eigen::Vector2d continuousProjection2d(const Eigen::Vector2d &v, const Eigen::Vector2d &dv, double bound)
{
    return (v.norm() < bound || (v.norm() == bound && dv.transpose() * v <= 0)) ? dv : (Eigen::Matrix2d::Ones() - v * v.transpose() / v.squaredNorm()) * dv;
}

