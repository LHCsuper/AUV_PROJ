#include "auv/controller/speed_controller.h"
#include "auv/model/auv_3dof_model.h"
#include "auv/simulator/logger_csv.h"
#include "auv/simulator/rk4_integrator.h"

#include <cmath>
#include <filesystem>
#include <iostream>

namespace {

bool near(double actual, double expected, double tolerance) {
    return std::abs(actual - expected) <= tolerance;
}

int expect(bool condition, const char* label) {
    if (!condition) {
        std::cerr << label << " failed\n";
        return 1;
    }
    return 0;
}

int expectNear(double actual, double expected, double tolerance, const char* label) {
    if (!near(actual, expected, tolerance)) {
        std::cerr << label << " failed: actual=" << actual
                  << " expected=" << expected
                  << " tolerance=" << tolerance << '\n';
        return 1;
    }
    return 0;
}

struct SpeedStepResult {
    double final_u{0.0};
    double final_error{0.0};
    double max_abs_x{0.0};
};

SpeedStepResult runSpeedStep(const std::filesystem::path& outputPath) {
    using auv::controller::PidController;
    using auv::controller::PidLimits;
    using auv::controller::SpeedController;
    using auv::controller::makeSurgeOnlyInput;
    using auv::model::Auv3DofModel;
    using auv::model::Auv3DofParams;
    using auv::model::ControlInput3DOF;
    using auv::model::State3DOF;
    using auv::simulator::CsvLogger;
    using auv::simulator::rk4Step;

    Auv3DofParams params;
    Auv3DofModel model(params);

    PidLimits limits;
    limits.integral_min = -10.0;
    limits.integral_max = 10.0;
    limits.output_min = -params.max_X;
    limits.output_max = params.max_X;

    SpeedController controller(PidController({90.0, 25.0, 0.0}, limits));

    CsvLogger logger(
        outputPath,
        {"time", "u", "u_d", "speed_error", "X_cmd", "x"}
    );

    State3DOF state;
    const double desired_u = 1.0;
    const double dt = 0.01;
    const double duration = 20.0;

    SpeedStepResult result;
    double time = 0.0;
    for (int step = 0; step <= static_cast<int>(duration / dt); ++step) {
        const double error = desired_u - state.u;
        const double thrust = controller.computeThrust(desired_u, state.u, dt);
        const ControlInput3DOF input = makeSurgeOnlyInput(thrust);

        logger.writeRow({time, state.u, desired_u, error, input.X, state.x});

        result.final_u = state.u;
        result.final_error = error;
        result.max_abs_x = std::max(result.max_abs_x, std::abs(input.X));

        state = rk4Step<State3DOF>(
            state,
            time,
            dt,
            [&](double, const State3DOF& s) {
                return model.derivative(s, input);
            },
            [&](State3DOF& s) {
                s = model.normalizeState(s);
            }
        );
        time += dt;
    }

    logger.close();
    return result;
}

}  // namespace

int main() {
    using auv::controller::PidController;
    using auv::controller::PidLimits;
    using auv::controller::SpeedController;
    using auv::controller::makeSurgeOnlyInput;

    int failures = 0;

    {
        SpeedController controller(PidController({2.0, 0.0, 0.0}));
        failures += expectNear(controller.computeThrust(1.0, 0.25, 0.1), 1.5, 1e-12, "speed proportional thrust");
        failures += expectNear(makeSurgeOnlyInput(3.0).X, 3.0, 1e-12, "surge input X");
        failures += expectNear(makeSurgeOnlyInput(3.0).N, 0.0, 1e-12, "surge input N zero");
    }

    const std::filesystem::path outputPath =
        std::filesystem::current_path() / "test_outputs" / "speed_step_response.csv";
    const SpeedStepResult result = runSpeedStep(outputPath);

    failures += expect(std::filesystem::exists(outputPath), "speed CSV exists");
    failures += expect(std::abs(result.final_error) < 0.03, "speed step converges");
    failures += expect(result.final_u > 0.9 && result.final_u < 1.1, "final speed near target");
    failures += expect(result.max_abs_x <= 80.0 + 1e-12, "thrust command saturated");

    {
        PidLimits limits;
        limits.output_min = -1.0;
        limits.output_max = 1.0;
        SpeedController controller(PidController({100.0, 0.0, 0.0}, limits));
        failures += expectNear(controller.computeThrust(1.0, 0.0, 0.1), 1.0, 1e-12, "speed output limit");
    }

    if (failures != 0) {
        return 1;
    }

    std::cout << "test_speed_controller passed\n";
    return 0;
}
