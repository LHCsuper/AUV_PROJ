#pragma once

#include "auv/controller/pid_controller.h"
#include "auv/model/auv_3dof_model.h"

namespace auv::controller {

class SpeedController {
public:
    explicit SpeedController(PidController pid);

    double computeThrust(double desired_u, double measured_u, double dt);
    void reset();

    const PidController& pid() const;
    PidController& pid();

private:
    PidController pid_;
};

auv::model::ControlInput3DOF makeSurgeOnlyInput(double thrust_x);

}  // namespace auv::controller
