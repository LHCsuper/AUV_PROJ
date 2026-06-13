#include "auv/guidance/reference_path.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace auv::guidance {

namespace {

constexpr double kMinSegmentLength = 1e-12;

void validatePointCount(std::size_t point_count, std::size_t min_count) {
    if (point_count < min_count) {
        throw std::invalid_argument("path point count is too small");
    }
}

double squaredDistanceToSegment(
    const LineSegment2D& segment,
    const Eigen::Vector2d& position,
    double& t,
    Eigen::Vector2d& closest
) {
    const Eigen::Vector2d delta = segment.end - segment.start;
    const double length_squared = delta.squaredNorm();
    if (length_squared <= kMinSegmentLength) {
        throw std::invalid_argument("path segment length must be positive");
    }

    t = (position - segment.start).dot(delta) / length_squared;
    t = std::clamp(t, 0.0, 1.0);
    closest = segment.start + t * delta;
    return (position - closest).squaredNorm();
}

}  // namespace

PolylinePath2D makeCirclePath(
    const Eigen::Vector2d& center,
    double radius,
    std::size_t point_count
) {
    if (radius <= 0.0) {
        throw std::invalid_argument("circle path radius must be positive");
    }
    validatePointCount(point_count, 3);

    PolylinePath2D path;
    path.closed = true;
    path.points.reserve(point_count);

    constexpr double two_pi = 6.28318530717958647692;
    for (std::size_t i = 0; i < point_count; ++i) {
        const double theta = two_pi * static_cast<double>(i) /
            static_cast<double>(point_count);
        path.points.emplace_back(
            center.x() + radius * std::cos(theta),
            center.y() + radius * std::sin(theta)
        );
    }

    return path;
}

PolylinePath2D makeSinePath(
    double x_start,
    double x_end,
    double amplitude,
    double wave_number,
    std::size_t point_count
) {
    if (x_end <= x_start) {
        throw std::invalid_argument("sine path x_end must be greater than x_start");
    }
    if (wave_number <= 0.0) {
        throw std::invalid_argument("sine path wave number must be positive");
    }
    validatePointCount(point_count, 2);

    PolylinePath2D path;
    path.closed = false;
    path.points.reserve(point_count);

    const double dx = (x_end - x_start) / static_cast<double>(point_count - 1);
    for (std::size_t i = 0; i < point_count; ++i) {
        const double x = x_start + dx * static_cast<double>(i);
        path.points.emplace_back(x, amplitude * std::sin(wave_number * x));
    }

    return path;
}

std::size_t segmentCount(const PolylinePath2D& path) {
    validatePointCount(path.points.size(), path.closed ? 3 : 2);
    return path.closed ? path.points.size() : path.points.size() - 1;
}

LineSegment2D segmentAt(const PolylinePath2D& path, std::size_t segment_index) {
    const std::size_t count = segmentCount(path);
    if (segment_index >= count) {
        throw std::out_of_range("path segment index is out of range");
    }

    const std::size_t next_index = (segment_index + 1) % path.points.size();
    return {path.points.at(segment_index), path.points.at(next_index)};
}

ClosestPathSegment closestSegment(
    const PolylinePath2D& path,
    const Eigen::Vector2d& position
) {
    const std::size_t count = segmentCount(path);

    ClosestPathSegment best;
    double best_squared_distance = std::numeric_limits<double>::infinity();

    for (std::size_t i = 0; i < count; ++i) {
        const LineSegment2D segment = segmentAt(path, i);
        double t = 0.0;
        Eigen::Vector2d closest{0.0, 0.0};
        const double squared_distance =
            squaredDistanceToSegment(segment, position, t, closest);

        if (squared_distance < best_squared_distance) {
            best.segment = segment;
            best.closest_point = closest;
            best.segment_index = i;
            best.segment_parameter = t;
            best.distance = std::sqrt(squared_distance);
            best_squared_distance = squared_distance;
        }
    }

    return best;
}

}  // namespace auv::guidance
