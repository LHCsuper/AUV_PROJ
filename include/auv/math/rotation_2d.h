#pragma once

#include <Eigen/Dense>

namespace auv::math {

Eigen::Matrix2d rotationBodyToWorld(double psi);
Eigen::Matrix2d rotationWorldToBody(double psi);

}  // namespace auv::math
