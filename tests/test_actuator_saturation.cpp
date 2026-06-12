#include "auv/actuator/saturation.h"
#include "auv/controller/pid_controller.h"
#include "auv/controller/speed_controller.h"

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
    using auv::actuator::SaturationLimits;
    using auv::actuator::applySaturation;
    using auv::actuator::saturateSymmetric;
    using auv::actuator::symmetricLimits;
    using auv::controller::PidController;
    using auv::controller::PidLimits;
    using auv::controller::SpeedController;

    int failures = 0;

    {
        const SaturationLimits limits{-2.0, 3.0};
        failures += expectNear(applySaturation(1.5, limits).value, 1.5, "inside saturation value");
        failures += expect(!applySaturation(1.5, limits).saturated, "inside saturation flag");
        failures += expectNear(applySaturation(4.0, limits).value, 3.0, "upper saturation value");
        failures += expect(applySaturation(4.0, limits).saturated, "upper saturation flag");
        failures += expectNear(applySaturation(-5.0, limits).value, -2.0, "lower saturation value");
        failures += expect(applySaturation(-5.0, limits).saturated, "lower saturation flag");
    }

    {
        const SaturationLimits limits = symmetricLimits(10.0);
        failures += expectNear(limits.min, -10.0, "symmetric min");
        failures += expectNear(limits.max, 10.0, "symmetric max");
        failures += expectNear(saturateSymmetric(20.0, 10.0), 10.0, "symmetric saturation upper");
        failures += expectNear(saturateSymmetric(-20.0, 10.0), -10.0, "symmetric saturation lower");
    }

    {
        bool rejected = false;
        try {
            (void)applySaturation(0.0, {1.0, -1.0});
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        failures += expect(rejected, "invalid saturation limits rejected");
    }

    {
        PidLimits limits;
        limits.integral_min = -100.0;
        limits.integral_max = 100.0;
        limits.output_min = -10.0;
        limits.output_max = 10.0;

        SpeedController controller(PidController({50.0, 20.0, 0.0}, limits));

        double command = 0.0;
        for (int i = 0; i < 200; ++i) {
            command = controller.computeThrust(100.0, 0.0, 0.01);
            failures += expectNear(command, 10.0, "unreachable speed command saturated");
        }

        failures += expectNear(controller.pid().integral(), 0.0, "anti-windup prevents integral growth");

        command = controller.computeThrust(0.0, 0.0, 0.01);
        failures += expectNear(command, 0.0, "command recovers after unreachable target");
        failures += expectNear(controller.pid().integral(), 0.0, "integral remains recovered");
    }

    if (failures != 0) {
        return 1;
    }

    std::cout << "test_actuator_saturation passed\n";
    return 0;
}
