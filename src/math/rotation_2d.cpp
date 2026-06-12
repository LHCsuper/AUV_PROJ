#include "auv/math/rotation_2d.h"

#include <cmath>

namespace auv::math {

Eigen::Matrix2d rotationBodyToWorld(double psi) {
    const double c = std::cos(psi);
    const double s = std::sin(psi);

    Eigen::Matrix2d rotation;
    rotation << c, -s,
                s,  c;
    return rotation;
}

Eigen::Matrix2d rotationWorldToBody(double psi) {
    return rotationBodyToWorld(psi).transpose();
}

}  // namespace auv::math
