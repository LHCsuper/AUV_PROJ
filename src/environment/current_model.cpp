#include "auv/environment/current_model.h"

#include "auv/math/rotation_2d.h"

namespace auv::environment {

Eigen::Vector2d currentWorldVector(const Current2D& current) {
    return {current.vx, current.vy};
}

Eigen::Vector2d currentBodyVector(const Current2D& current, double psi) {
    return auv::math::rotationWorldToBody(psi) * currentWorldVector(current);
}

}  // namespace auv::environment
