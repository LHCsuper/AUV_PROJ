#pragma once

#include "auv/guidance/los_guidance.h"

#include <Eigen/Dense>

#include <cstddef>
#include <vector>

namespace auv::guidance {

struct PolylinePath2D {
    std::vector<Eigen::Vector2d> points;
    bool closed{false};
};

struct ClosestPathSegment {
    LineSegment2D segment;
    Eigen::Vector2d closest_point{0.0, 0.0};
    std::size_t segment_index{0};
    double segment_parameter{0.0};
    double distance{0.0};
};

PolylinePath2D makeCirclePath(
    const Eigen::Vector2d& center,
    double radius,
    std::size_t point_count
);

PolylinePath2D makeSinePath(
    double x_start,
    double x_end,
    double amplitude,
    double wave_number,
    std::size_t point_count
);

std::size_t segmentCount(const PolylinePath2D& path);
LineSegment2D segmentAt(const PolylinePath2D& path, std::size_t segment_index);
ClosestPathSegment closestSegment(
    const PolylinePath2D& path,
    const Eigen::Vector2d& position
);

}  // namespace auv::guidance
