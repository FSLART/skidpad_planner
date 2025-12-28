#pragma once
#include <Eigen/Dense>

Eigen::Vector2d unit_vector_from_angle(double yaw);
Eigen::Vector2d rotate(const Eigen::Vector2d& v, double yaw);
