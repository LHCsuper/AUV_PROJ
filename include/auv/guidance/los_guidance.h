#pragma once

#include <Eigen/Dense>

namespace auv::guidance {

struct LineSegment2D {
    Eigen::Vector2d start{0.0, 0.0};
    Eigen::Vector2d end{1.0, 0.0};
};

struct LosGuidanceResult {
    double path_angle{0.0};
    double cross_track_error{0.0};
    double desired_heading{0.0};
};

class LineOfSightGuidance {
public:
    explicit LineOfSightGuidance(double lookahead_distance);

    LosGuidanceResult compute(
        const LineSegment2D& path,
        const Eigen::Vector2d& position
    ) const;

    double lookaheadDistance() const;

private:
    double lookahead_distance_{1.0};
};

double pathAngle(const LineSegment2D& path);
double crossTrackError(const LineSegment2D& path, const Eigen::Vector2d& position);

}  // namespace auv::guidance
