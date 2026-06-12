#pragma once

#include "auv/environment/current_model.h"
#include "auv/model/auv_3dof_params.h"

namespace auv::model {

struct State3DOF {
    double x{0.0};
    double y{0.0};
    double psi{0.0};
    double u{0.0};
    double v{0.0};
    double r{0.0};
};

struct ControlInput3DOF {
    double X{0.0};
    double N{0.0};
};

struct Auv3DofModelOptions {
    bool enable_coupling{false};
};

class Auv3DofModel {
public:
    explicit Auv3DofModel(
        Auv3DofParams params = {},
        Auv3DofModelOptions options = {}
    );

    State3DOF derivative(const State3DOF& state, const ControlInput3DOF& input) const;
    State3DOF derivative(
        const State3DOF& state,
        const ControlInput3DOF& input,
        const auv::environment::Current2D& current
    ) const;
    State3DOF normalizeState(const State3DOF& state) const;

    Auv3DofParams params() const;
    Auv3DofModelOptions options() const;

private:
    ControlInput3DOF applyActuatorLimits(const ControlInput3DOF& input) const;

    Auv3DofParams params_;
    Auv3DofModelOptions options_;
};

State3DOF operator+(const State3DOF& lhs, const State3DOF& rhs);
State3DOF operator-(const State3DOF& lhs, const State3DOF& rhs);
State3DOF operator*(const State3DOF& state, double scalar);
State3DOF operator*(double scalar, const State3DOF& state);

}  // namespace auv::model
