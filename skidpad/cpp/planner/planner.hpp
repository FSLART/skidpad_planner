#pragma once

#include <Eigen/Dense>
#include <vector>
#include "path_types.hpp"

class Planner
{
public:
    Planner(int mode = 0);

    Path calculate_path(
        const std::vector<Eigen::MatrixXd>& cones_by_type,
        const Eigen::Vector2d& car_position,
        const Eigen::Vector2d& car_direction);

private:
    int mode_;
};
