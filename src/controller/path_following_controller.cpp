#include "auv/controller/path_following_controller.h"

#include "auv/math/angle_utils.h"

namespace auv::controller {

LosPidPathFollower::LosPidPathFollower(
    auv::guidance::LineOfSightGuidance los,
    SpeedController speed_controller,
    PidController heading_controller
) : los_(los),
    speed_controller_(std::move(speed_controller)),
    heading_controller_(std::move(heading_controller)) {}

PathFollowingOutput LosPidPathFollower::compute(
    const auv::model::State3DOF& state,
    const PathFollowingReference& reference,
    double dt
) {
    const auto los_result = los_.compute(
        reference.path,
        Eigen::Vector2d{state.x, state.y}
    );

    const double heading_error =
        auv::math::wrapToPi(los_result.desired_heading - state.psi);

    const double thrust_x =
        speed_controller_.computeThrust(reference.desired_speed, state.u, dt);
    const double yaw_moment_n = heading_controller_.update(heading_error, dt);

    auv::model::ControlInput3DOF input;
    input.X = thrust_x;
    input.N = yaw_moment_n;

    PathFollowingDebug debug;
    debug.desired_heading = los_result.desired_heading;
    debug.heading_error = heading_error;
    debug.cross_track_error = los_result.cross_track_error;
    debug.thrust_x = thrust_x;
    debug.yaw_moment_n = yaw_moment_n;

    return {input, debug};
}

void LosPidPathFollower::reset() {
    speed_controller_.reset();
    heading_controller_.reset();
}

}  // namespace auv::controller
