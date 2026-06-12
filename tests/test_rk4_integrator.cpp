#include "auv/math/angle_utils.h"
#include "auv/simulator/rk4_integrator.h"

#include <Eigen/Dense>

#include <cmath>
#include <iostream>
#include <stdexcept>

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

}  // namespace

int main() {
    using auv::math::pi;
    using auv::math::wrapToPi;
    using auv::simulator::rk4Step;

    int failures = 0;

    {
        double x = 0.0;
        double t = 0.0;
        const double dt = 0.01;
        for (int i = 0; i < 1000; ++i) {
            x = rk4Step<double>(x, t, dt, [](double, double) {
                return 1.0;
            });
            t += dt;
        }
        failures += expectNear(x, 10.0, 1e-12, "dx/dt=1 integration");
    }

    {
        double x = 1.0;
        double t = 0.0;
        const double dt = 0.01;
        for (int i = 0; i < 100; ++i) {
            x = rk4Step<double>(x, t, dt, [](double, double state) {
                return -state;
            });
            t += dt;
        }
        failures += expectNear(x, std::exp(-1.0), 1e-10, "dx/dt=-x integration");
    }

    {
        Eigen::Vector2d state{0.0, 1.0};
        double t = 0.0;
        const double dt = 0.01;
        for (int i = 0; i < 100; ++i) {
            state = rk4Step<Eigen::Vector2d>(state, t, dt, [](double, const Eigen::Vector2d&) {
                return Eigen::Vector2d{1.0, -1.0};
            });
            t += dt;
        }
        failures += expectNear(state.x(), 1.0, 1e-12, "vector x integration");
        failures += expectNear(state.y(), 0.0, 1e-12, "vector y integration");
    }

    {
        Eigen::Vector2d state{pi() - 0.01, 1.0};
        state = rk4Step<Eigen::Vector2d>(
            state,
            0.0,
            0.1,
            [](double, const Eigen::Vector2d& s) {
                return Eigen::Vector2d{s.y(), 0.0};
            },
            [](Eigen::Vector2d& s) {
                s.x() = wrapToPi(s.x());
            }
        );
        failures += expect(state.x() >= -pi() && state.x() <= pi(), "wrapped angle range");
        failures += expect(state.x() < 0.0, "wrapped angle crosses pi boundary");
    }

    {
        bool rejected = false;
        try {
            (void)rk4Step<double>(0.0, 0.0, 0.0, [](double, double) {
                return 1.0;
            });
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        failures += expect(rejected, "dt zero rejection");
    }

    if (failures != 0) {
        return 1;
    }

    std::cout << "test_rk4_integrator passed\n";
    return 0;
}
