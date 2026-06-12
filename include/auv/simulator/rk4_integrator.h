#pragma once

#include <stdexcept>
#include <type_traits>
#include <utility>

namespace auv::simulator {

struct NoStateNormalization {
    template <typename State>
    void operator()(State&) const {}
};

template <
    typename State,
    typename Derivative,
    typename Normalize = NoStateNormalization
>
State rk4Step(
    const State& state,
    double time,
    double dt,
    Derivative&& derivative,
    Normalize&& normalize = {}
) {
    if (dt <= 0.0) {
        throw std::invalid_argument("RK4 step requires dt > 0");
    }

    const State k1 = derivative(time, state);
    const State k2 = derivative(time + 0.5 * dt, state + k1 * (0.5 * dt));
    const State k3 = derivative(time + 0.5 * dt, state + k2 * (0.5 * dt));
    const State k4 = derivative(time + dt, state + k3 * dt);

    State next = state + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (dt / 6.0);
    normalize(next);
    return next;
}

}  // namespace auv::simulator
