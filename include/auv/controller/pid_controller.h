#pragma once

#include <limits>

namespace auv::controller {

struct PidGains {
    double kp{0.0};
    double ki{0.0};
    double kd{0.0};
};

struct PidLimits {
    double integral_min{-std::numeric_limits<double>::infinity()};
    double integral_max{std::numeric_limits<double>::infinity()};
    double output_min{-std::numeric_limits<double>::infinity()};
    double output_max{std::numeric_limits<double>::infinity()};
};

class PidController {
public:
    PidController() = default;
    PidController(PidGains gains, PidLimits limits = {});

    double update(double error, double dt);
    void reset();

    void setGains(PidGains gains);
    void setLimits(PidLimits limits);

    PidGains gains() const;
    PidLimits limits() const;
    double integral() const;
    double previousError() const;
    bool hasPreviousError() const;

private:
    double clamp(double value, double lower, double upper) const;
    void validateLimits() const;

    PidGains gains_{};
    PidLimits limits_{};
    double integral_{0.0};
    double previous_error_{0.0};
    bool has_previous_error_{false};
};

}  // namespace auv::controller
