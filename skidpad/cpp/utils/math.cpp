#include "math.hpp"
#include <cmath>

Eigen::Vector2d unit_vector_from_angle(double yaw)
{
    return Eigen::Vector2d(std::cos(yaw), std::sin(yaw));
}

Eigen::Vector2d rotate(const Eigen::Vector2d& v, double yaw)
{
    Eigen::Matrix2d R;
    R << std::cos(yaw), -std::sin(yaw),
         std::sin(yaw),  std::cos(yaw);
    return R * v;
}
