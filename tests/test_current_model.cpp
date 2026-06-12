#include "auv/environment/current_model.h"
#include "auv/math/angle_utils.h"
#include "auv/model/auv_3dof_model.h"
#include "auv/simulator/rk4_integrator.h"

#include <cmath>
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

auv::model::State3DOF integrateWithCurrent(
    const auv::model::Auv3DofModel& model,
    auv::model::State3DOF state,
    auv::model::ControlInput3DOF input,
    auv::environment::Current2D current,
    double duration,
    double dt
) {
    double time = 0.0;
    const int steps = static_cast<int>(duration / dt);
    for (int i = 0; i < steps; ++i) {
        state = auv::simulator::rk4Step<auv::model::State3DOF>(
            state,
            time,
            dt,
            [&](double, const auv::model::State3DOF& s) {
                return model.derivative(s, input, current);
            },
            [&](auv::model::State3DOF& s) {
                s = model.normalizeState(s);
            }
        );
        time += dt;
    }
    return state;
}

}  // namespace

int main() {
    using auv::environment::Current2D;
    using auv::environment::currentBodyVector;
    using auv::math::pi;
    using auv::model::Auv3DofModel;
    using auv::model::ControlInput3DOF;
    using auv::model::State3DOF;

    int failures = 0;

    Auv3DofModel model;

    {
        State3DOF state;
        state.u = 1.2;
        state.v = -0.1;
        state.r = 0.2;
        state.psi = 0.3;
        const State3DOF no_current = model.derivative(state, {});
        const State3DOF zero_current = model.derivative(state, {}, {});
        failures += expectNear(zero_current.x, no_current.x, 1e-12, "zero current xdot");
        failures += expectNear(zero_current.y, no_current.y, 1e-12, "zero current ydot");
        failures += expectNear(zero_current.u, no_current.u, 1e-12, "zero current udot");
        failures += expectNear(zero_current.v, no_current.v, 1e-12, "zero current vdot");
        failures += expectNear(zero_current.r, no_current.r, 1e-12, "zero current rdot");
    }

    {
        const Current2D current{0.2, 0.0};
        const Eigen::Vector2d body0 = currentBodyVector(current, 0.0);
        const Eigen::Vector2d body90 = currentBodyVector(current, pi() / 2.0);
        failures += expectNear(body0.x(), 0.2, 1e-12, "current body psi 0 x");
        failures += expectNear(body0.y(), 0.0, 1e-12, "current body psi 0 y");
        failures += expectNear(body90.x(), 0.0, 1e-12, "current body psi pi/2 x");
        failures += expectNear(body90.y(), -0.2, 1e-12, "current body psi pi/2 y");
    }

    {
        State3DOF final = integrateWithCurrent(
            model,
            {},
            {},
            {0.2, 0.1},
            10.0,
            0.01
        );
        failures += expect(final.x > 2.0, "drift x follows current and hydrodynamic drag");
        failures += expect(final.y > 1.0, "drift y follows current and hydrodynamic drag");
        failures += expect(final.u > 0.0, "current-induced relative damping changes surge speed");
        failures += expect(final.v > 0.0, "current-induced relative damping changes sway speed");
    }

    {
        State3DOF state;
        state.u = 1.0;
        const State3DOF no_current = model.derivative(state, {});
        const State3DOF following_current = model.derivative(state, {}, {1.0, 0.0});
        const State3DOF opposing_current = model.derivative(state, {}, {-1.0, 0.0});
        failures += expect(std::abs(following_current.u) < std::abs(no_current.u), "following current reduces relative damping");
        failures += expect(std::abs(opposing_current.u) > std::abs(no_current.u), "opposing current increases relative damping");
    }

    {
        State3DOF state;
        state.u = 1.0;
        const State3DOF no_current = integrateWithCurrent(
            model,
            state,
            {},
            {},
            5.0,
            0.01
        );
        const State3DOF cross_current = integrateWithCurrent(
            model,
            state,
            {},
            {0.0, 0.2},
            5.0,
            0.01
        );
        failures += expect(std::abs(cross_current.y) > std::abs(no_current.y) + 0.5, "cross current causes lateral drift");
    }

    if (failures != 0) {
        return 1;
    }

    std::cout << "test_current_model passed\n";
    return 0;
}
