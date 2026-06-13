#include "auv/controller/path_following_controller.h"
#include "auv/guidance/los_guidance.h"
#include "auv/math/angle_utils.h"
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

struct PathFollowingResult {
    double initial_cross_track_error{0.0};
    double final_cross_track_error{0.0};
    double final_heading_error{0.0};
    double final_speed_error{0.0};
    double max_abs_x{0.0};
    double max_abs_n{0.0};
};

PathFollowingResult runStraightLineFollowing(const std::filesystem::path& outputPath) {
    using auv::controller::LosPidPathFollower;
    using auv::controller::PathFollowingReference;
    using auv::controller::PidController;
    using auv::controller::PidLimits;
    using auv::controller::SpeedController;
    using auv::guidance::LineOfSightGuidance;
    using auv::guidance::LineSegment2D;
    using auv::model::Auv3DofModel;
    using auv::model::Auv3DofParams;
    using auv::model::State3DOF;
    using auv::simulator::CsvLogger;
    using auv::simulator::rk4Step;

    Auv3DofParams params;
    Auv3DofModel model(params);

    PidLimits speed_limits;
    speed_limits.integral_min = -10.0;
    speed_limits.integral_max = 10.0;
    speed_limits.output_min = -params.max_X;
    speed_limits.output_max = params.max_X;

    PidLimits heading_limits;
    heading_limits.integral_min = -2.0;
    heading_limits.integral_max = 2.0;
    heading_limits.output_min = -params.max_N;
    heading_limits.output_max = params.max_N;

    LosPidPathFollower controller(
        LineOfSightGuidance(8.0),
        SpeedController(PidController({90.0, 25.0, 0.0}, speed_limits)),
        PidController({10.0, 0.0, 8.0}, heading_limits)
    );

    PathFollowingReference reference;
    reference.path = LineSegment2D{{0.0, 0.0}, {120.0, 0.0}};
    reference.desired_speed = 1.0;

    State3DOF state;
    state.x = 0.0;
    state.y = 5.0;
    state.psi = 0.0;

    CsvLogger logger(
        outputPath,
        {
            "time",
            "x",
            "y",
            "psi",
            "u",
            "v",
            "r",
            "x_ref",
            "y_ref",
            "psi_d",
            "cross_track_error",
            "heading_error",
            "X_cmd",
            "N_cmd"
        }
    );

    const double dt = 0.02;
    const double duration = 90.0;
    double time = 0.0;

    PathFollowingResult result;
    bool first_sample = true;

    for (int step = 0; step <= static_cast<int>(duration / dt); ++step) {
        const auto output = controller.compute(state, reference, dt);

        if (first_sample) {
            result.initial_cross_track_error = output.debug.cross_track_error;
            first_sample = false;
        }

        result.final_cross_track_error = output.debug.cross_track_error;
        result.final_heading_error = output.debug.heading_error;
        result.final_speed_error = reference.desired_speed - state.u;
        result.max_abs_x = std::max(result.max_abs_x, std::abs(output.input.X));
        result.max_abs_n = std::max(result.max_abs_n, std::abs(output.input.N));

        logger.writeRow({
            time,
            state.x,
            state.y,
            state.psi,
            state.u,
            state.v,
            state.r,
            state.x,
            0.0,
            output.debug.desired_heading,
            output.debug.cross_track_error,
            output.debug.heading_error,
            output.input.X,
            output.input.N
        });

        state = rk4Step<State3DOF>(
            state,
            time,
            dt,
            [&](double, const State3DOF& s) {
                return model.derivative(s, output.input);
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
    int failures = 0;

    const std::filesystem::path outputPath =
        std::filesystem::current_path() / "test_outputs" / "straight_line_path_following.csv";
    const PathFollowingResult result = runStraightLineFollowing(outputPath);

    failures += expect(std::filesystem::exists(outputPath), "path following CSV exists");
    failures += expect(std::abs(result.initial_cross_track_error) > 4.0, "initial cross-track error is large");
    failures += expect(std::abs(result.final_cross_track_error) < 0.25, "cross-track error converges");
    failures += expect(std::abs(result.final_cross_track_error) < std::abs(result.initial_cross_track_error), "cross-track error decreases");
    failures += expect(std::abs(result.final_heading_error) < 0.1, "heading error converges");
    failures += expect(std::abs(result.final_speed_error) < 0.05, "speed error remains small");
    failures += expect(result.max_abs_x <= 80.0 + 1e-12, "surge command respects saturation");
    failures += expect(result.max_abs_n <= 20.0 + 1e-12, "yaw command respects saturation");

    if (failures != 0) {
        return 1;
    }

    std::cout << "test_path_following passed\n";
    return 0;
}
