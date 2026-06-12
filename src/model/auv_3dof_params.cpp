#include "auv/model/auv_3dof_params.h"

#include <stdexcept>

namespace auv::model {

namespace {

void requirePositive(double value, const char* name) {
    if (value <= 0.0) {
        throw std::invalid_argument(std::string(name) + " must be positive");
    }
}

void requireNonNegative(double value, const char* name) {
    if (value < 0.0) {
        throw std::invalid_argument(std::string(name) + " must be non-negative");
    }
}

}  // namespace

void validateAuv3DofParams(const Auv3DofParams& params) {
    requirePositive(params.m11, "m11");
    requirePositive(params.m22, "m22");
    requirePositive(params.m33, "m33");

    requireNonNegative(params.d_u, "d_u");
    requireNonNegative(params.d_v, "d_v");
    requireNonNegative(params.d_r, "d_r");

    requireNonNegative(params.d_uu, "d_uu");
    requireNonNegative(params.d_vv, "d_vv");
    requireNonNegative(params.d_rr, "d_rr");

    requirePositive(params.max_X, "max_X");
    requirePositive(params.max_N, "max_N");
    requirePositive(params.thruster_scale, "thruster_scale");
}

Auv3DofParams scaledAuv3DofParams(
    const Auv3DofParams& nominal,
    const Auv3DofUncertaintyScale& scale
) {
    requirePositive(scale.inertia_scale, "inertia_scale");
    requirePositive(scale.damping_scale, "damping_scale");
    requirePositive(scale.thruster_scale, "thruster_scale");
    validateAuv3DofParams(nominal);

    Auv3DofParams scaled = nominal;

    scaled.m11 *= scale.inertia_scale;
    scaled.m22 *= scale.inertia_scale;
    scaled.m33 *= scale.inertia_scale;

    scaled.d_u *= scale.damping_scale;
    scaled.d_v *= scale.damping_scale;
    scaled.d_r *= scale.damping_scale;
    scaled.d_uu *= scale.damping_scale;
    scaled.d_vv *= scale.damping_scale;
    scaled.d_rr *= scale.damping_scale;

    scaled.thruster_scale *= scale.thruster_scale;

    validateAuv3DofParams(scaled);
    return scaled;
}

}  // namespace auv::model
