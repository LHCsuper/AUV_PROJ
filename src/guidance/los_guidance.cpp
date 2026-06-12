#include "auv/guidance/los_guidance.h"

#include "auv/math/angle_utils.h"

#include <cmath>
#include <stdexcept>

namespace auv::guidance {

namespace {

Eigen::Vector2d pathDelta(const LineSegment2D& path) {
    return path.end - path.start;
}

void validatePath(const LineSegment2D& path) {
    if (pathDelta(path).norm() <= 1e-12) {
        throw std::invalid_argument("LOS path segment length must be positive");
    }
}

}  // namespace

LineOfSightGuidance::LineOfSightGuidance(double lookahead_distance)
    : lookahead_distance_(lookahead_distance) {
    if (lookahead_distance_ <= 0.0) {
        throw std::invalid_argument("LOS lookahead distance must be positive");
    }
}

LosGuidanceResult LineOfSightGuidance::compute(
    const LineSegment2D& path,
    const Eigen::Vector2d& position
) const {
    const double gamma = pathAngle(path);
    const double error = crossTrackError(path, position);
    const double desired_heading = auv::math::wrapToPi(
        gamma - std::atan2(error, lookahead_distance_)
    );

    return {gamma, error, desired_heading};
}

double LineOfSightGuidance::lookaheadDistance() const {
    return lookahead_distance_;
}

double pathAngle(const LineSegment2D& path) {
    validatePath(path);
    const Eigen::Vector2d delta = pathDelta(path);
    return auv::math::wrapToPi(std::atan2(delta.y(), delta.x()));
}

double crossTrackError(
    const LineSegment2D& path,
    const Eigen::Vector2d& position
) {
    validatePath(path);
    const double gamma = pathAngle(path);
    const Eigen::Vector2d relative = position - path.start;
    return -relative.x() * std::sin(gamma) + relative.y() * std::cos(gamma);
}

}  // namespace auv::guidance
