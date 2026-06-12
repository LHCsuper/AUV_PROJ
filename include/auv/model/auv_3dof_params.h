#pragma once

namespace auv::model {

struct Auv3DofParams {
    double m11{60.0};
    double m22{80.0};
    double m33{12.0};

    double d_u{18.0};
    double d_v{30.0};
    double d_r{8.0};

    double d_uu{25.0};
    double d_vv{45.0};
    double d_rr{12.0};

    double max_X{80.0};
    double max_N{20.0};

    double thruster_scale{1.0};
};

struct Auv3DofUncertaintyScale {
    double inertia_scale{1.0};
    double damping_scale{1.0};
    double thruster_scale{1.0};
};

void validateAuv3DofParams(const Auv3DofParams& params);
Auv3DofParams scaledAuv3DofParams(
    const Auv3DofParams& nominal,
    const Auv3DofUncertaintyScale& scale
);

}  // namespace auv::model
