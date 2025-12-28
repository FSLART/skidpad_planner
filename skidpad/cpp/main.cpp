#include <iostream>
#include <Eigen/Dense>

#include "planner/planner.hpp"
#include "utils/math.hpp"

int main()
{
    Planner planner(4);

    // temp/imaginary/fake car state
    Eigen::Vector2d car_position(0.0, 0.0);
    double yaw = 0.3;
    Eigen::Vector2d car_direction = unit_vector_from_angle(yaw);

    // temp/imaginary/fake cones (empty for now)
    std::vector<Eigen::MatrixXd> cones_by_type(5);

    Path path = planner.calculate_path(
        cones_by_type,
        car_position,
        car_direction);

    for (const auto& p : path) {
        std::cout << p.s << " "
                  << p.x << " "
                  << p.y << " "
                  << p.curvature << "\n";
    }

    return 0;
}
