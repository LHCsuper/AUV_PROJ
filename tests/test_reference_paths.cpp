#include "auv/guidance/reference_path.h"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

bool near(double actual, double expected, double tolerance) {
    return std::abs(actual - expected) <= tolerance;
}

int expect(bool condition, const char* label) {
    if (!condition) {
        std::cerr << label << " failed\n";
        return 1;
    }
    return 0;
}

int expectNear(double actual, double expected, double tolerance, const char* label) {
    if (!near(actual, expected, tolerance)) {
        std::cerr << label << " failed: actual=" << actual
                  << " expected=" << expected
                  << " tolerance=" << tolerance << '\n';
        return 1;
    }
    return 0;
}

template <typename Fn>
int expectThrows(Fn&& fn, const char* label) {
    try {
        fn();
    } catch (const std::exception&) {
        return 0;
    }

    std::cerr << label << " failed: expected exception\n";
    return 1;
}

}  // namespace

int main() {
    using auv::guidance::closestSegment;
    using auv::guidance::makeCirclePath;
    using auv::guidance::makeSinePath;
    using auv::guidance::segmentAt;
    using auv::guidance::segmentCount;

    int failures = 0;

    const auto circle = makeCirclePath(Eigen::Vector2d{1.0, -2.0}, 10.0, 40);
    failures += expect(circle.closed, "circle path is closed");
    failures += expect(circle.points.size() == 40, "circle point count");
    failures += expect(segmentCount(circle) == 40, "circle segment count");
    failures += expectNear((circle.points.front() - Eigen::Vector2d{11.0, -2.0}).norm(), 0.0, 1e-12, "circle first point");

    const auto lastCircleSegment = segmentAt(circle, 39);
    failures += expectNear((lastCircleSegment.end - circle.points.front()).norm(), 0.0, 1e-12, "circle last segment closes");

    const auto circleClosest = closestSegment(circle, Eigen::Vector2d{11.0, 1.0});
    failures += expect(circleClosest.segment_index < segmentCount(circle), "circle closest segment index");
    failures += expect(circleClosest.distance < 3.1, "circle closest distance");

    const auto sine = makeSinePath(0.0, 20.0, 3.0, 0.4, 81);
    failures += expect(!sine.closed, "sine path is open");
    failures += expect(sine.points.size() == 81, "sine point count");
    failures += expect(segmentCount(sine) == 80, "sine segment count");
    failures += expectNear(sine.points.front().x(), 0.0, 1e-12, "sine first x");
    failures += expectNear(sine.points.back().x(), 20.0, 1e-12, "sine last x");

    const auto sineClosest = closestSegment(sine, Eigen::Vector2d{10.0, 0.0});
    failures += expect(sineClosest.segment_index < segmentCount(sine), "sine closest segment index");
    failures += expect(sineClosest.segment_parameter >= 0.0 && sineClosest.segment_parameter <= 1.0, "sine closest parameter");

    failures += expectThrows(
        [] { (void)makeCirclePath(Eigen::Vector2d{0.0, 0.0}, -1.0, 20); },
        "invalid circle radius"
    );
    failures += expectThrows(
        [] { (void)makeSinePath(1.0, 0.0, 1.0, 0.2, 20); },
        "invalid sine x range"
    );
    failures += expectThrows(
        [] { (void)makeSinePath(0.0, 1.0, 1.0, 0.2, 1); },
        "invalid sine point count"
    );

    if (failures != 0) {
        return 1;
    }

    std::cout << "test_reference_paths passed\n";
    return 0;
}
