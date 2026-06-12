#include "auv/controller/speed_controller.h"

namespace auv::controller {

SpeedController::SpeedController(PidController pid) : pid_(pid) {}

double SpeedController::computeThrust(
    double desired_u,
    double measured_u,
    double dt
) {
    const double error = desired_u - measured_u;
    return pid_.update(error, dt);
}

void SpeedController::reset() {
    pid_.reset();
}

const PidController& SpeedController::pid() const {
    return pid_;
}

PidController& SpeedController::pid() {
    return pid_;
}

auv::model::ControlInput3DOF makeSurgeOnlyInput(double thrust_x) {
    auv::model::ControlInput3DOF input;
    input.X = thrust_x;
    input.N = 0.0;
    return input;
}

}  // namespace auv::controller
