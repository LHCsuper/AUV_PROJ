#pragma once

#include "auv/controller/pid_controller.h"
#include "auv/controller/speed_controller.h"
#include "auv/guidance/los_guidance.h"
#include "auv/model/auv_3dof_model.h"

namespace auv::controller {

struct PathFollowingReference {
    auv::guidance::LineSegment2D path;
    double desired_speed{1.0};
};

struct PathFollowingDebug {
    double desired_heading{0.0};
    double heading_error{0.0};
    double cross_track_error{0.0};
    double thrust_x{0.0};
    double yaw_moment_n{0.0};
};

struct PathFollowingOutput {
    auv::model::ControlInput3DOF input;
    PathFollowingDebug debug;
};

class LosPidPathFollower {
public:
    LosPidPathFollower(
        auv::guidance::LineOfSightGuidance los,
        SpeedController speed_controller,
        PidController heading_controller
    );

    PathFollowingOutput compute(
        const auv::model::State3DOF& state,
        const PathFollowingReference& reference,
        double dt
    );

    void reset();

private:
    auv::guidance::LineOfSightGuidance los_;
    SpeedController speed_controller_;
    PidController heading_controller_;
};

}  // namespace auv::controller
