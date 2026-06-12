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

auv::model::State3DOF integrate(
    const auv::model::Auv3DofModel& model,
    auv::model::State3DOF state,
    auv::model::ControlInput3DOF input,
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
                return model.derivative(s, input);
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
    using auv::math::pi;
    using auv::model::Auv3DofModel;
    using auv::model::Auv3DofModelOptions;
    using auv::model::Auv3DofParams;
    using auv::model::ControlInput3DOF;
    using auv::model::State3DOF;

    int failures = 0;

    Auv3DofParams params;
    Auv3DofModel model(params);

    {
        State3DOF state;
        state.u = 1.0;
        const State3DOF final = integrate(model, state, {}, 8.0, 0.01);
        failures += expect(final.u > 0.0, "surge speed remains positive while decaying");
        failures += expect(final.u < state.u, "surge speed decays without input");
        failures += expect(std::abs(final.v) < 1e-12, "sway remains zero without coupling/input");
        failures += expect(std::abs(final.r) < 1e-12, "yaw rate remains zero without input");
    }

    {
        State3DOF state;
        const ControlInput3DOF input{20.0, 0.0};
        const State3DOF final = integrate(model, state, input, 25.0, 0.01);
        const double residual =
            input.X - params.d_u * final.u - params.d_uu * std::abs(final.u) * final.u;
        failures += expect(final.u > 0.0, "constant positive thrust gives positive surge speed");
        failures += expect(std::abs(residual) < 0.05, "surge speed approaches steady state");
    }

    {
        State3DOF state;
        const ControlInput3DOF input{0.0, 4.0};
        const State3DOF final = integrate(model, state, input, 20.0, 0.01);
        const double residual =
            input.N - params.d_r * final.r - params.d_rr * std::abs(final.r) * final.r;
        failures += expect(final.r > 0.0, "constant positive yaw moment gives positive yaw rate");
        failures += expect(final.psi > 0.0, "positive yaw rate increases heading");
        failures += expect(std::abs(residual) < 0.05, "yaw rate approaches steady state");
    }

    {
        State3DOF state;
        state.psi = pi() - 0.01;
        state.r = 1.0;
        const State3DOF final = integrate(model, state, {}, 0.1, 0.01);
        failures += expect(final.psi >= -pi() && final.psi <= pi(), "psi normalized after integration");
        failures += expect(final.psi < 0.0, "psi wraps across pi boundary");
    }

    {
        State3DOF a{1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
        State3DOF b{0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
        const State3DOF result = (a + b) * 2.0 - b;
        failures += expectNear(result.x, 2.5, 1e-12, "state operator x");
        failures += expectNear(result.r, 12.5, 1e-12, "state operator r");
    }

    {
        Auv3DofModelOptions options;
        options.enable_coupling = true;
        Auv3DofModel coupled(params, options);
        State3DOF state;
        state.u = 1.0;
        state.v = 0.2;
        state.r = 0.1;
        const State3DOF d = coupled.derivative(state, {});
        failures += expect(d.u != model.derivative(state, {}).u, "coupling changes surge derivative");
        failures += expect(d.r != model.derivative(state, {}).r, "coupling changes yaw derivative");
    }

    if (failures != 0) {
        return 1;
    }

    std::cout << "test_auv_3dof_model passed\n";
    return 0;
}
