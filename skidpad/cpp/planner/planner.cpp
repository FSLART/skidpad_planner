#include "planner.hpp"
#include <cmath>

Planner::Planner(int mode)
    : mode_(mode)
{
}

Path Planner::calculate_path(
    const std::vector<Eigen::MatrixXd>& cones_by_type,
    const Eigen::Vector2d& car_position,
    const Eigen::Vector2d& car_direction)
{
    Path path;

    // generate a straight line in front of the car
    const int N = 50;
    const double step = 0.5;

    for (int i = 0; i < N; ++i) {
        double s = i * step;

        Eigen::Vector2d p =
            car_position + s * car_direction;

        PathPoint pt;
        pt.s = s;
        pt.x = p.x();
        pt.y = p.y();
        pt.curvature = 0.0;

        path.push_back(pt);
    }

    return path;
}
