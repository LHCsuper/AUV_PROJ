#include "auv/model/auv_3dof_model.h"

#include "auv/actuator/saturation.h"
#include "auv/environment/current_model.h"
#include "auv/math/angle_utils.h"

#include <cmath>

namespace auv::model {

Auv3DofModel::Auv3DofModel(
    Auv3DofParams params,
    Auv3DofModelOptions options
) : params_(params), options_(options) {
    validateAuv3DofParams(params_);
}

State3DOF Auv3DofModel::derivative(
    const State3DOF& state,
    const ControlInput3DOF& input
) const {
    return derivative(state, input, {});
}

State3DOF Auv3DofModel::derivative(
    const State3DOF& state,
    const ControlInput3DOF& input,
    const auv::environment::Current2D& current
) const {
    const ControlInput3DOF limited = applyActuatorLimits(input);
    const double X = params_.thruster_scale * limited.X;
    const double N = params_.thruster_scale * limited.N;

    State3DOF d;

    const double c = std::cos(state.psi);
    const double s = std::sin(state.psi);
    d.x = state.u * c - state.v * s + current.vx;
    d.y = state.u * s + state.v * c + current.vy;
    d.psi = state.r;

    const Eigen::Vector2d body_current =
        auv::environment::currentBodyVector(current, state.psi);
    const double u_r = state.u - body_current.x();
    const double v_r = state.v - body_current.y();

    const double surge_damping = params_.d_u * u_r + params_.d_uu * std::abs(u_r) * u_r;
    const double sway_damping = params_.d_v * v_r + params_.d_vv * std::abs(v_r) * v_r;
    const double yaw_damping = params_.d_r * state.r + params_.d_rr * std::abs(state.r) * state.r;

    double surge_coupling = 0.0;
    double sway_coupling = 0.0;
    double yaw_coupling = 0.0;

    if (options_.enable_coupling) {
        surge_coupling = params_.m22 * v_r * state.r;
        sway_coupling = -params_.m11 * u_r * state.r;
        yaw_coupling = (params_.m11 - params_.m22) * u_r * v_r;
    }

    d.u = (X - surge_damping + surge_coupling) / params_.m11;
    d.v = (-sway_damping + sway_coupling) / params_.m22;
    d.r = (N - yaw_damping + yaw_coupling) / params_.m33;

    return d;
}

State3DOF Auv3DofModel::normalizeState(const State3DOF& state) const {
    State3DOF normalized = state;
    normalized.psi = auv::math::wrapToPi(normalized.psi);
    return normalized;
}

Auv3DofParams Auv3DofModel::params() const {
    return params_;
}

Auv3DofModelOptions Auv3DofModel::options() const {
    return options_;
}

ControlInput3DOF Auv3DofModel::applyActuatorLimits(
    const ControlInput3DOF& input
) const {
    ControlInput3DOF limited;
    limited.X = auv::actuator::saturateSymmetric(input.X, params_.max_X);
    limited.N = auv::actuator::saturateSymmetric(input.N, params_.max_N);
    return limited;
}

State3DOF operator+(const State3DOF& lhs, const State3DOF& rhs) {
    return {
        lhs.x + rhs.x,
        lhs.y + rhs.y,
        lhs.psi + rhs.psi,
        lhs.u + rhs.u,
        lhs.v + rhs.v,
        lhs.r + rhs.r
    };
}

State3DOF operator-(const State3DOF& lhs, const State3DOF& rhs) {
    return {
        lhs.x - rhs.x,
        lhs.y - rhs.y,
        lhs.psi - rhs.psi,
        lhs.u - rhs.u,
        lhs.v - rhs.v,
        lhs.r - rhs.r
    };
}

State3DOF operator*(const State3DOF& state, double scalar) {
    return {
        state.x * scalar,
        state.y * scalar,
        state.psi * scalar,
        state.u * scalar,
        state.v * scalar,
        state.r * scalar
    };
}

State3DOF operator*(double scalar, const State3DOF& state) {
    return state * scalar;
}

}  // namespace auv::model
