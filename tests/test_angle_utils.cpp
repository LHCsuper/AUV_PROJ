#include "auv/math/angle_utils.h"

#include <cmath>
#include <iostream>

namespace {

bool near(double actual, double expected, double tolerance = 1e-12) {
    return std::abs(actual - expected) <= tolerance;
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
    using auv::math::deg2rad;
    using auv::math::pi;
    using auv::math::rad2deg;
    using auv::math::wrapToPi;

    int failures = 0;

    failures += expectNear(deg2rad(180.0), pi(), "deg2rad(180)");
    failures += expectNear(rad2deg(pi()), 180.0, "rad2deg(pi)");
    failures += expectNear(wrapToPi(pi() + 0.1), -pi() + 0.1, "wrap pi + 0.1");
    failures += expectNear(wrapToPi(-pi() - 0.1), pi() - 0.1, "wrap -pi - 0.1");
    failures += expectNear(wrapToPi(0.0), 0.0, "wrap zero");
    failures += expectNear(wrapToPi(5.0 * pi() / 2.0), pi() / 2.0, "wrap 5pi/2");

    if (failures != 0) {
        return 1;
    }

    std::cout << "test_angle_utils passed\n";
    return 0;
}
