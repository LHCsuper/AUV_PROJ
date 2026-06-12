#include "auv/model/heading_model.h"

#include "auv/math/angle_utils.h"

#include <stdexcept>

namespace auv::model {

HeadingModel::HeadingModel(HeadingModelParams params) : params_(params) {
    if (params_.inertia_z <= 0.0) {
        throw std::invalid_argument("Heading inertia_z must be positive");
    }
    if (params_.damping_r < 0.0) {
        throw std::invalid_argument("Heading damping_r must be non-negative");
    }
}

HeadingDerivative HeadingModel::derivative(
    const HeadingState& state,
    double yaw_moment
) const {
    HeadingDerivative derivative;
    derivative.psi_dot = state.r;
    derivative.r_dot = (yaw_moment - params_.damping_r * state.r) / params_.inertia_z;
    return derivative;
}

HeadingState HeadingModel::eulerStep(
    const HeadingState& state,
    double yaw_moment,
    double dt
) const {
    if (dt <= 0.0) {
        throw std::invalid_argument("Heading eulerStep requires dt > 0");
    }

    const HeadingDerivative d = derivative(state, yaw_moment);

    HeadingState next;
    next.psi = auv::math::wrapToPi(state.psi + d.psi_dot * dt);
    next.r = state.r + d.r_dot * dt;
    return next;
}

HeadingModelParams HeadingModel::params() const {
    return params_;
}

}  // namespace auv::model
