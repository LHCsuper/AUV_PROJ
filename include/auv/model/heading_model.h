#pragma once

namespace auv::model {

struct HeadingState {
    double psi{0.0};
    double r{0.0};
};

struct HeadingDerivative {
    double psi_dot{0.0};
    double r_dot{0.0};
};

struct HeadingModelParams {
    double inertia_z{5.0};
    double damping_r{2.0};
};

class HeadingModel {
public:
    explicit HeadingModel(HeadingModelParams params = {});

    HeadingDerivative derivative(const HeadingState& state, double yaw_moment) const;
    HeadingState eulerStep(const HeadingState& state, double yaw_moment, double dt) const;

    HeadingModelParams params() const;

private:
    HeadingModelParams params_;
};

}  // namespace auv::model
