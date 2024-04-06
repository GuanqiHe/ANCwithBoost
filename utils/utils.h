#pragma once

#include <Eigen/Dense>

Eigen::Vector2d discreteProjection2d(const Eigen::Vector2d &v, double bound);

Eigen::Vector2d discreteProjection2dOut(const Eigen::Vector2d &v, double bound);

Eigen::Vector2d continuousProjection2d(const Eigen::Vector2d &v, const Eigen::Vector2d &dv, double bound);

