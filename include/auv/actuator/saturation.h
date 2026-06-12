#pragma once

namespace auv::actuator {

struct SaturationLimits {
    double min{0.0};
    double max{0.0};
};

struct SaturationResult {
    double value{0.0};
    bool saturated{false};
};

SaturationLimits symmetricLimits(double absolute_limit);
void validateSaturationLimits(const SaturationLimits& limits);
SaturationResult applySaturation(double command, const SaturationLimits& limits);
double saturate(double command, const SaturationLimits& limits);
double saturateSymmetric(double command, double absolute_limit);

}  // namespace auv::actuator
