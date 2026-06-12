#include "auv/controller/pid_controller.h"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

bool near(double actual, double expected, double tolerance = 1e-12) {
    return std::abs(actual - expected) <= tolerance;
}

int expect(bool condition, const char* label) {
    if (!condition) {
        std::cerr << label << " failed\n";
        return 1;
    }
    return 0;
}

int expectNear(double actual, double expected, const char* label) {
    if (!near(actual, expected)) {
        std::cerr << label << " failed: actual=" << actual
                  << " expected=" << expected << '\n';
        return 1;
    }
    return 0;
}

}  // namespace

int main() {
    using auv::controller::PidController;
    using auv::controller::PidGains;
    using auv::controller::PidLimits;

    int failures = 0;

    {
        PidController pid({2.0, 0.0, 0.0});
        failures += expectNear(pid.update(3.0, 0.1), 6.0, "proportional output");
    }

    {
        PidLimits limits;
        limits.integral_min = -0.5;
        limits.integral_max = 0.5;
        PidController pid({0.0, 1.0, 0.0}, limits);
        pid.update(1.0, 1.0);
        failures += expectNear(pid.integral(), 0.5, "integral upper clamp");
        pid.update(-2.0, 1.0);
        failures += expectNear(pid.integral(), -0.5, "integral lower clamp");
    }

    {
        PidLimits limits;
        limits.output_min = -1.0;
        limits.output_max = 1.0;
        PidController pid({10.0, 0.0, 0.0}, limits);
        failures += expectNear(pid.update(1.0, 0.1), 1.0, "output upper clamp");
        failures += expectNear(pid.update(-1.0, 0.1), -1.0, "output lower clamp");
    }

    {
        PidLimits limits;
        limits.output_min = -1.0;
        limits.output_max = 1.0;
        PidController pid({0.0, 10.0, 0.0}, limits);
        pid.update(1.0, 1.0);
        failures += expectNear(pid.integral(), 0.0, "anti-windup holds integral");
    }

    {
        PidController pid({1.0, 1.0, 1.0});
        pid.update(2.0, 0.5);
        pid.reset();
        failures += expectNear(pid.integral(), 0.0, "reset integral");
        failures += expect(!pid.hasPreviousError(), "reset previous error flag");
    }

    {
        PidController pid({1.0, 0.0, 0.0});
        bool rejected = false;
        try {
            pid.update(1.0, 0.0);
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        failures += expect(rejected, "dt zero rejection");
    }

    {
        bool rejected = false;
        try {
            PidLimits limits;
            limits.output_min = 2.0;
            limits.output_max = 1.0;
            PidController pid({1.0, 0.0, 0.0}, limits);
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        failures += expect(rejected, "invalid limit rejection");
    }

    if (failures != 0) {
        return 1;
    }

    std::cout << "test_pid_controller passed\n";
    return 0;
}
