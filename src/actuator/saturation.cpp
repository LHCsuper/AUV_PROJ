#include "auv/actuator/saturation.h"

#include <algorithm>
#include <stdexcept>

namespace auv::actuator {

SaturationLimits symmetricLimits(double absolute_limit) {
    if (absolute_limit <= 0.0) {
        throw std::invalid_argument("Saturation absolute_limit must be positive");
    }
    return {-absolute_limit, absolute_limit};
}

void validateSaturationLimits(const SaturationLimits& limits) {
    if (limits.min > limits.max) {
        throw std::invalid_argument("Saturation min must be <= max");
    }
}

SaturationResult applySaturation(double command, const SaturationLimits& limits) {
    validateSaturationLimits(limits);

    const double value = std::clamp(command, limits.min, limits.max);
    return {value, value != command};
}

double saturate(double command, const SaturationLimits& limits) {
    return applySaturation(command, limits).value;
}

double saturateSymmetric(double command, double absolute_limit) {
    return saturate(command, symmetricLimits(absolute_limit));
}

}  // namespace auv::actuator
