#include "auv/math/angle_utils.h"

#include <cmath>

namespace auv::math {

double pi() {
    return std::acos(-1.0);
}

double twoPi() {
    return 2.0 * pi();
}

double deg2rad(double degrees) {
    return degrees * pi() / 180.0;
}

double rad2deg(double radians) {
    return radians * 180.0 / pi();
}

double wrapToPi(double radians) {
    const double period = twoPi();
    double wrapped = std::fmod(radians + pi(), period);
    if (wrapped < 0.0) {
        wrapped += period;
    }
    return wrapped - pi();
}

}  // namespace auv::math
