#include "auv/controller/path_following_controller.h"
#include "auv/guidance/reference_path.h"
#include "auv/math/angle_utils.h"
#include "auv/model/auv_3dof_model.h"
#include "auv/simulator/logger_csv.h"
#include "auv/simulator/rk4_integrator.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <string>

namespace {

int expect(bool condition, const char* label) {
    if (!condition) {
        std::cerr << label << " failed\n";
        return 1;
    }
    return 0;
}

struct CurvedPathResult {
    double initial_abs_cross_track_error{0.0};
    double final_abs_cross_track_error{0.0};
    double max_abs_cross_track_error_after_settling{0.0};
    double final_speed_error{0.0};
    double max_abs_heading_step{0.0};
    double max_abs_x{0.0};
    double max_abs_n{0.0};
};

auv::controller::LosPidPathFollower makeController(const auv::model::Auv3DofParams& params) {
    using auv::controller::LosPidPathFollower;
    using auv::controller::PidController;
    using auv::controller::PidLimits;
    using auv::controller::SpeedController;
    using auv::guidance::LineOfSightGuidance;

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

    return LosPidPathFollower(
        LineOfSightGuidance(6.0),
        SpeedController(PidController({90.0, 25.0, 0.0}, speed_limits)),
        PidController({12.0, 0.0, 9.0}, heading_limits)
    );
}

CurvedPathResult runPolylineFollowing(
    const auv::guidance::PolylinePath2D& path,
    auv::model::State3DOF initial_state,
    double desired_speed,
    double duration,
    const std::filesystem::path& output_path
) {
    using auv::controller::PathFollowingReference;
    using auv::guidance::closestSegment;
    using auv::model::Auv3DofModel;
    using auv::model::Auv3DofParams;
    using auv::model::State3DOF;
    using auv::simulator::CsvLogger;
    using auv::simulator::rk4Step;

    Auv3DofParams params;
    Auv3DofModel model(params);
    auto controller = makeController(params);

    CsvLogger logger(
        output_path,
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
            "N_cmd",
            "segment_index"
        }
    );

    constexpr double dt = 0.02;
    const int total_steps = static_cast<int>(duration / dt);
    const int settling_steps = static_cast<int>(0.25 * static_cast<double>(total_steps));

    State3DOF state = initial_state;
    CurvedPathResult result;
    bool first_sample = true;
    bool has_previous_heading = false;
    double previous_heading = 0.0;
    double time = 0.0;

    for (int step = 0; step <= total_steps; ++step) {
        const auto closest = closestSegment(path, Eigen::Vector2d{state.x, state.y});

        PathFollowingReference reference;
        reference.path = closest.segment;
        reference.desired_speed = desired_speed;

        const auto output = controller.compute(state, reference, dt);
        const double abs_cross_track_error = std::abs(output.debug.cross_track_error);

        if (first_sample) {
            result.initial_abs_cross_track_error = abs_cross_track_error;
            first_sample = false;
        }

        if (has_previous_heading) {
            const double heading_step = auv::math::wrapToPi(
                output.debug.desired_heading - previous_heading
            );
            result.max_abs_heading_step =
                std::max(result.max_abs_heading_step, std::abs(heading_step));
        }
        previous_heading = output.debug.desired_heading;
        has_previous_heading = true;

        if (step >= settling_steps) {
            result.max_abs_cross_track_error_after_settling =
                std::max(result.max_abs_cross_track_error_after_settling, abs_cross_track_error);
        }

        result.final_abs_cross_track_error = abs_cross_track_error;
        result.final_speed_error = desired_speed - state.u;
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
            closest.closest_point.x(),
            closest.closest_point.y(),
            output.debug.desired_heading,
            output.debug.cross_track_error,
            output.debug.heading_error,
            output.input.X,
            output.input.N,
            static_cast<double>(closest.segment_index)
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

void printResult(const std::string& name, const CurvedPathResult& result) {
    std::cout << name
              << " initial_abs_ey=" << result.initial_abs_cross_track_error
              << " final_abs_ey=" << result.final_abs_cross_track_error
              << " max_abs_ey_after_settling=" << result.max_abs_cross_track_error_after_settling
              << " final_speed_error=" << result.final_speed_error
              << " max_abs_heading_step=" << result.max_abs_heading_step
              << '\n';
}

}  // namespace

int main() {
    using auv::guidance::makeCirclePath;
    using auv::guidance::makeSinePath;
    using auv::math::deg2rad;
    using auv::model::State3DOF;

    int failures = 0;
    const std::filesystem::path output_dir =
        std::filesystem::current_path() / "test_outputs";

    State3DOF circle_initial;
    circle_initial.x = 30.0;
    circle_initial.y = 0.0;
    circle_initial.psi = deg2rad(90.0);

    const auto circle = makeCirclePath(Eigen::Vector2d{0.0, 0.0}, 25.0, 120);
    const auto circle_result = runPolylineFollowing(
        circle,
        circle_initial,
        0.6,
        120.0,
        output_dir / "circle_path_following.csv"
    );
    printResult("circle", circle_result);

    failures += expect(
        std::filesystem::exists(output_dir / "circle_path_following.csv"),
        "circle path CSV exists"
    );
    failures += expect(circle_result.initial_abs_cross_track_error > 4.0, "circle initial error is meaningful");
    failures += expect(circle_result.final_abs_cross_track_error < 1.0, "circle final cross-track error is bounded");
    failures += expect(circle_result.max_abs_cross_track_error_after_settling < 2.0, "circle settled cross-track error is bounded");
    failures += expect(circle_result.max_abs_heading_step < 0.35, "circle desired heading changes smoothly");
    failures += expect(std::abs(circle_result.final_speed_error) < 0.08, "circle speed remains near target");
    failures += expect(circle_result.max_abs_x <= 80.0 + 1e-12, "circle surge command respects saturation");
    failures += expect(circle_result.max_abs_n <= 20.0 + 1e-12, "circle yaw command respects saturation");

    State3DOF sine_initial;
    sine_initial.x = 0.0;
    sine_initial.y = -4.0;
    sine_initial.psi = deg2rad(25.0);

    const auto sine = makeSinePath(0.0, 100.0, 8.0, 0.08, 160);
    const auto sine_result = runPolylineFollowing(
        sine,
        sine_initial,
        0.8,
        120.0,
        output_dir / "s_curve_path_following.csv"
    );
    printResult("s_curve", sine_result);

    failures += expect(
        std::filesystem::exists(output_dir / "s_curve_path_following.csv"),
        "S path CSV exists"
    );
    failures += expect(sine_result.initial_abs_cross_track_error > 3.0, "S path initial error is meaningful");
    failures += expect(sine_result.final_abs_cross_track_error < 1.0, "S path final cross-track error is bounded");
    failures += expect(sine_result.max_abs_cross_track_error_after_settling < 2.5, "S path settled cross-track error is bounded");
    failures += expect(sine_result.max_abs_heading_step < 0.35, "S path desired heading changes smoothly");
    failures += expect(std::abs(sine_result.final_speed_error) < 0.08, "S path speed remains near target");
    failures += expect(sine_result.max_abs_x <= 80.0 + 1e-12, "S path surge command respects saturation");
    failures += expect(sine_result.max_abs_n <= 20.0 + 1e-12, "S path yaw command respects saturation");

    if (failures != 0) {
        return 1;
    }

    std::cout << "test_curved_path_following passed\n";
    return 0;
}
