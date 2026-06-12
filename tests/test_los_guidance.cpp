#include "auv/guidance/los_guidance.h"
#include "auv/math/angle_utils.h"

#include <Eigen/Dense>

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
    using auv::guidance::LineOfSightGuidance;
    using auv::guidance::LineSegment2D;
    using auv::guidance::crossTrackError;
    using auv::guidance::pathAngle;
    using auv::math::pi;

    int failures = 0;

    const LineSegment2D east_path{{0.0, 0.0}, {10.0, 0.0}};

    {
        LineOfSightGuidance los(5.0);
        const auto result = los.compute(east_path, {2.0, 0.0});
        failures += expectNear(result.path_angle, 0.0, "east path angle");
        failures += expectNear(result.cross_track_error, 0.0, "on-path cross-track error");
        failures += expectNear(result.desired_heading, 0.0, "on-path desired heading");
    }

    {
        LineOfSightGuidance los(5.0);
        const auto left = los.compute(east_path, {2.0, 1.0});
        const auto right = los.compute(east_path, {2.0, -1.0});
        failures += expect(left.cross_track_error > 0.0, "left side positive cross-track error");
        failures += expect(right.cross_track_error < 0.0, "right side negative cross-track error");
        failures += expect(left.desired_heading < 0.0, "left side correction turns right");
        failures += expect(right.desired_heading > 0.0, "right side correction turns left");
    }

    {
        LineOfSightGuidance short_los(2.0);
        LineOfSightGuidance long_los(10.0);
        const auto short_result = short_los.compute(east_path, {2.0, 1.0});
        const auto long_result = long_los.compute(east_path, {2.0, 1.0});
        failures += expect(std::abs(short_result.desired_heading) > std::abs(long_result.desired_heading), "larger lookahead is smoother");
    }

    {
        const LineSegment2D north_path{{0.0, 0.0}, {0.0, 10.0}};
        failures += expectNear(pathAngle(north_path), pi() / 2.0, "north path angle");
        failures += expectNear(crossTrackError(north_path, {-1.0, 2.0}), 1.0, "north path left error");
    }

    {
        const LineSegment2D near_pi_path{{0.0, 0.0}, {-10.0, 0.001}};
        LineOfSightGuidance los(5.0);
        const auto result = los.compute(near_pi_path, {0.0, -1.0});
        failures += expect(result.desired_heading >= -pi() && result.desired_heading <= pi(), "desired heading wrapped");
    }

    {
        bool rejected = false;
        try {
            LineOfSightGuidance invalid(0.0);
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        failures += expect(rejected, "invalid lookahead rejected");
    }

    {
        bool rejected = false;
        try {
            (void)pathAngle({{1.0, 1.0}, {1.0, 1.0}});
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        failures += expect(rejected, "zero-length path rejected");
    }

    if (failures != 0) {
        return 1;
    }

    std::cout << "test_los_guidance passed\n";
    return 0;
}
