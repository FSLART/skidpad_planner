#pragma once
#include <Eigen/Dense>
#include <vector>

struct PathPoint
{
    double s;        // distance along path
    double x;
    double y;
    double curvature;
};

using Path = std::vector<PathPoint>;
