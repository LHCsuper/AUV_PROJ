#pragma once

#include <Eigen/Dense>

namespace auv::environment {

struct Current2D {
    double vx{0.0};
    double vy{0.0};
};

Eigen::Vector2d currentWorldVector(const Current2D& current);
Eigen::Vector2d currentBodyVector(const Current2D& current, double psi);

}  // namespace auv::environment
