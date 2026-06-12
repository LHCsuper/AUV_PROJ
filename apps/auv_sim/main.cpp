#include <iostream>

#ifdef AUV_HAS_EIGEN
#include <Eigen/Dense>
#endif

int main() {
    std::cout << "AUV simulation started\n";

#ifdef AUV_HAS_EIGEN
    const Eigen::Vector2d current{0.3, 0.1};
    std::cout << "Eigen check: current norm = " << current.norm() << '\n';
#else
    std::cout << "Eigen check: not enabled in this build\n";
#endif

    return 0;
}
