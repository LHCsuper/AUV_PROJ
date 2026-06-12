#include "auv/model/auv_3dof_params.h"

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
    using auv::model::Auv3DofParams;
    using auv::model::Auv3DofUncertaintyScale;
    using auv::model::scaledAuv3DofParams;
    using auv::model::validateAuv3DofParams;

    int failures = 0;

    {
        Auv3DofParams params;
        validateAuv3DofParams(params);
        failures += expect(params.m11 > 0.0, "default m11 positive");
        failures += expect(params.m22 > 0.0, "default m22 positive");
        failures += expect(params.m33 > 0.0, "default m33 positive");
        failures += expect(params.max_X > 0.0, "default max_X positive");
        failures += expect(params.max_N > 0.0, "default max_N positive");
    }

    {
        Auv3DofParams params;
        Auv3DofUncertaintyScale scale;
        scale.inertia_scale = 1.2;
        scale.damping_scale = 1.3;
        scale.thruster_scale = 0.9;

        const Auv3DofParams scaled = scaledAuv3DofParams(params, scale);
        failures += expectNear(scaled.m11, params.m11 * 1.2, "scaled m11");
        failures += expectNear(scaled.m22, params.m22 * 1.2, "scaled m22");
        failures += expectNear(scaled.m33, params.m33 * 1.2, "scaled m33");
        failures += expectNear(scaled.d_u, params.d_u * 1.3, "scaled d_u");
        failures += expectNear(scaled.d_rr, params.d_rr * 1.3, "scaled d_rr");
        failures += expectNear(scaled.thruster_scale, params.thruster_scale * 0.9, "scaled thruster");
        failures += expectNear(scaled.max_X, params.max_X, "max_X unchanged");
        failures += expectNear(scaled.max_N, params.max_N, "max_N unchanged");
    }

    {
        bool rejected = false;
        try {
            Auv3DofParams params;
            params.m11 = 0.0;
            validateAuv3DofParams(params);
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        failures += expect(rejected, "invalid inertia rejected");
    }

    {
        bool rejected = false;
        try {
            Auv3DofParams params;
            params.d_u = -1.0;
            validateAuv3DofParams(params);
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        failures += expect(rejected, "negative damping rejected");
    }

    {
        bool rejected = false;
        try {
            Auv3DofParams params;
            Auv3DofUncertaintyScale scale;
            scale.damping_scale = 0.0;
            (void)scaledAuv3DofParams(params, scale);
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        failures += expect(rejected, "invalid scale rejected");
    }

    if (failures != 0) {
        return 1;
    }

    std::cout << "test_auv_3dof_params passed\n";
    return 0;
}
