#include "auv/math/angle_utils.h"
#include "auv/math/rotation_2d.h"

#include <Eigen/Dense>

#include <cmath>
#include <iostream>

namespace {

bool near(double actual, double expected, double tolerance = 1e-12) {
    return std::abs(actual - expected) <= tolerance;
}

int expectVectorNear(
    const Eigen::Vector2d& actual,
    const Eigen::Vector2d& expected,
    const char* label
) {
    if (!near((actual - expected).norm(), 0.0)) {
        std::cerr << label << " failed: actual=[" << actual.transpose()
                  << "] expected=[" << expected.transpose() << "]\n";
        return 1;
    }
    return 0;
}

int expectMatrixNear(
    const Eigen::Matrix2d& actual,
    const Eigen::Matrix2d& expected,
    const char* label
) {
    if (!near((actual - expected).norm(), 0.0)) {
        std::cerr << label << " failed\nactual=\n" << actual
                  << "\nexpected=\n" << expected << '\n';
        return 1;
    }
    return 0;
}

}  // namespace

int main() {
    using auv::math::pi;
    using auv::math::rotationBodyToWorld;
    using auv::math::rotationWorldToBody;

    int failures = 0;

    const Eigen::Vector2d bodyForward{1.0, 0.0};
    const Eigen::Vector2d bodyRight{0.0, 1.0};

    failures += expectVectorNear(
        rotationBodyToWorld(0.0) * bodyForward,
        Eigen::Vector2d{1.0, 0.0},
        "psi=0 body x"
    );

    failures += expectVectorNear(
        rotationBodyToWorld(pi() / 2.0) * bodyForward,
        Eigen::Vector2d{0.0, 1.0},
        "psi=pi/2 body x"
    );

    failures += expectVectorNear(
        rotationBodyToWorld(pi() / 2.0) * bodyRight,
        Eigen::Vector2d{-1.0, 0.0},
        "psi=pi/2 body y"
    );

    const double psi = 0.37;
    failures += expectMatrixNear(
        rotationWorldToBody(psi) * rotationBodyToWorld(psi),
        Eigen::Matrix2d::Identity(),
        "world/body inverse"
    );

    if (failures != 0) {
        return 1;
    }

    std::cout << "test_rotation_2d passed\n";
    return 0;
}
