#include <Eigen/Dense>
#include <iostream>

int main() {
    Eigen::Vector2d v(1, 2);
    std::cout << v.norm() << std::endl;
    return 0;
}
