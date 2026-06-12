#include "auv/controller/pid_controller.h"

#include <algorithm>
#include <stdexcept>

namespace auv::controller {

PidController::PidController(PidGains gains, PidLimits limits)
    : gains_(gains), limits_(limits) {
    validateLimits();
}

double PidController::update(double error, double dt) {
    if (dt <= 0.0) {
        throw std::invalid_argument("PID update requires dt > 0");
    }

    const double previous_integral = integral_;
    integral_ = clamp(
        integral_ + error * dt,
        limits_.integral_min,
        limits_.integral_max
    );

    double derivative = 0.0;
    if (has_previous_error_) {
        derivative = (error - previous_error_) / dt;
    }

    const double raw_output =
        gains_.kp * error + gains_.ki * integral_ + gains_.kd * derivative;
    const double output = clamp(raw_output, limits_.output_min, limits_.output_max);

    const bool saturated = output != raw_output;
    const bool saturation_would_worsen =
        (raw_output > limits_.output_max && error > 0.0) ||
        (raw_output < limits_.output_min && error < 0.0);

    if (saturated && saturation_would_worsen) {
        integral_ = previous_integral;
    }

    previous_error_ = error;
    has_previous_error_ = true;

    if (saturated && saturation_would_worsen) {
        const double corrected_raw_output =
            gains_.kp * error + gains_.ki * integral_ + gains_.kd * derivative;
        return clamp(corrected_raw_output, limits_.output_min, limits_.output_max);
    }

    return output;
}

void PidController::reset() {
    integral_ = 0.0;
    previous_error_ = 0.0;
    has_previous_error_ = false;
}

void PidController::setGains(PidGains gains) {
    gains_ = gains;
}

void PidController::setLimits(PidLimits limits) {
    limits_ = limits;
    validateLimits();
    integral_ = clamp(integral_, limits_.integral_min, limits_.integral_max);
}

PidGains PidController::gains() const {
    return gains_;
}

PidLimits PidController::limits() const {
    return limits_;
}

double PidController::integral() const {
    return integral_;
}

double PidController::previousError() const {
    return previous_error_;
}

bool PidController::hasPreviousError() const {
    return has_previous_error_;
}

double PidController::clamp(double value, double lower, double upper) const {
    return std::clamp(value, lower, upper);
}

void PidController::validateLimits() const {
    if (limits_.integral_min > limits_.integral_max) {
        throw std::invalid_argument("PID integral_min must be <= integral_max");
    }
    if (limits_.output_min > limits_.output_max) {
        throw std::invalid_argument("PID output_min must be <= output_max");
    }
}

}  // namespace auv::controller
