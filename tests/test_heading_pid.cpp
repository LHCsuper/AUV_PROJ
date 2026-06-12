#include "auv/controller/pid_controller.h"
#include "auv/math/angle_utils.h"
#include "auv/model/heading_model.h"
#include "auv/simulator/logger_csv.h"

#include <cmath>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

namespace {

bool near(double actual, double expected, double tolerance = 1e-9) {
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

struct SimulationResult {
    double final_error{0.0};
    double max_abs_moment{0.0};
    double final_psi{0.0};
    double final_r{0.0};
};

SimulationResult runHeadingStepResponse(const std::filesystem::path& outputPath) {
    using auv::controller::PidController;
    using auv::controller::PidLimits;
    using auv::math::deg2rad;
    using auv::math::wrapToPi;
    using auv::model::HeadingModel;
    using auv::model::HeadingState;
    using auv::simulator::CsvLogger;

    HeadingModel model({5.0, 2.0});

    PidLimits limits;
    limits.output_min = -2.0;
    limits.output_max = 2.0;
    PidController headingPid({4.0, 0.0, 6.0}, limits);

    CsvLogger logger(
        outputPath,
        {"time", "psi", "psi_d", "heading_error", "r", "N_cmd"}
    );

    HeadingState state;
    const double psi_d = deg2rad(30.0);
    const double dt = 0.01;
    const double duration = 12.0;

    SimulationResult result;
    for (int step = 0; step <= static_cast<int>(duration / dt); ++step) {
        const double time = step * dt;
        const double error = wrapToPi(psi_d - state.psi);
        const double moment = headingPid.update(error, dt);

        logger.writeRow({time, state.psi, psi_d, error, state.r, moment});

        result.max_abs_moment = std::max(result.max_abs_moment, std::abs(moment));
        result.final_error = error;
        result.final_psi = state.psi;
        result.final_r = state.r;

        state = model.eulerStep(state, moment, dt);
    }

    logger.close();
    return result;
}

}  // namespace

int main() {
    using auv::math::deg2rad;
    using auv::math::wrapToPi;
    using auv::model::HeadingModel;
    using auv::model::HeadingState;

    int failures = 0;

    {
        HeadingModel model({5.0, 2.0});
        HeadingState state;
        const auto derivative = model.derivative(state, 1.0);
        failures += expectNear(derivative.psi_dot, 0.0, "open-loop psi_dot");
        failures += expectNear(derivative.r_dot, 0.2, "open-loop r_dot direction");
    }

    {
        const double error = wrapToPi(deg2rad(-179.0) - deg2rad(179.0));
        failures += expect(std::abs(error - deg2rad(2.0)) < deg2rad(0.001), "179 to -179 wrap");
    }

    const std::filesystem::path outputPath =
        std::filesystem::current_path() / "test_outputs" / "heading_step_response.csv";
    const SimulationResult step = runHeadingStepResponse(outputPath);

    failures += expect(std::filesystem::exists(outputPath), "heading CSV exists");
    failures += expect(std::abs(step.final_error) < deg2rad(2.0), "heading step converges");
    failures += expect(std::abs(step.final_r) < 0.05, "heading rate settles");
    failures += expect(step.max_abs_moment <= 2.0 + 1e-12, "yaw moment saturated");

    {
        bool rejected = false;
        try {
            HeadingModel invalid({0.0, 2.0});
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        failures += expect(rejected, "invalid heading inertia rejected");
    }

    if (failures != 0) {
        return 1;
    }

    std::cout << "test_heading_pid passed\n";
    return 0;
}
