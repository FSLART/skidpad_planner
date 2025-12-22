#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <iostream>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;
using Eigen::MatrixXd;
using Eigen::Vector2d;

// ------------------------------------------------------
// SETTINGS
// ------------------------------------------------------
constexpr bool USE_SPLINE = false;
constexpr double ROTATE_CONES = 0.0;
constexpr int ANIMATION_SPEED_MS = 5;

Vector2d random_point(-20.0, -20.0);

// ------------------------------------------------------
// REFERENCE CONES
// ------------------------------------------------------
MatrixXd example_cones(4, 2);
example_cones << 0, -1.5, 0,  1.5, 3, -1.5, 3,  1.5;

// Center
Vector2d center = example_cones.colwise().mean();

// Translate to origin
MatrixXd translated = example_cones.rowwise() - center.transpose();

// Rotation
double theta = -ROTATE_CONES * M_PI / 180.0;
MatrixXd R(2,2);
R << cos(theta), -sin(theta),
     sin(theta),  cos(theta);

// Apply rotation
MatrixXd rotated_cones = (translated * R.transpose()).rowwise() + center.transpose();

// ------------------------------------------------------
// LOAD TRACK DATA
// ------------------------------------------------------
MatrixXd loadCSV(const std::string& path) {
  std::ifstream file(path);
  std::vector<double> values;
  int rows = 0;

  std::string line;
  while (std::getline(file, line)) {
    std::stringstream ss(line);
    double val;
    while (ss >> val) {
      values.push_back(val);
      if (ss.peek() == ',') ss.ignore();
    }
    rows++;
  }

  return Eigen::Map<MatrixXd>(values.data(), rows, 2);
}

// ------------------------------------------------------
// TRACK ROTATION
// ------------------------------------------------------
Vector2d mid12 = (rotated_cones.row(0) + rotated_cones.row(1)) / 2.0;
Vector2d mid34 = (rotated_cones.row(2) + rotated_cones.row(3)) / 2.0;

Vector2d direction = mid34 - mid12;
double track_angle = atan2(direction.y(), direction.x());

// Rotate skidpad
MatrixXd R_track(2,2);
R_track << cos(track_angle), -sin(track_angle), sin(track_angle),  cos(track_angle);

MatrixXd rotated_skidpad = skidpad * R_track.transpose();
rotated_skidpad.rowwise() += center.transpose();

// ------------------------------------------------------
// CONE PAIR GENERATOR
// ------------------------------------------------------
MatrixXd make_cone_pair(const Vector2d& center, double spacing, double width, double theta)
{
  MatrixXd local(4,2);
  local << 0, -spacing, 0,  spacing, width, -spacing, width,  spacing;

  MatrixXd R(2,2);
  R << cos(theta), -sin(theta), sin(theta),  cos(theta);

  return (local * R.transpose()).rowwise() + center.transpose();
}

// ------------------------------------------------------
// CALCULATE CLOSEST POINT
// ------------------------------------------------------
int closest_index(const Vector2d& p, const MatrixXd& path) {
    double min_dist = 1e9;
    int idx = 0;

    for (int i = 0; i < path.rows(); i++) {
        double d = (path.row(i).transpose() - p).norm();
        if (d < min_dist) {
            min_dist = d;
            idx = i;
        }
    }
    return idx;
}

// ------------------------------------------------------
// ANIMATION LOOP
// ------------------------------------------------------
plt::figure();
plt::axis("equal");

for (int i = 1; i < current_path.rows(); i++) {
    std::vector<double> x, y;

    for (int j = std::max(0, i - 20); j < i; j++) {
        x.push_back(current_path(j,0));
        y.push_back(current_path(j,1));
    }

    plt::clf();
    plt::plot(x, y, "b.");

    // Heading vector
    Vector2d dir = current_path.row(i) - current_path.row(i - 1);
    dir.normalize();
    dir *= 5.0;

    plt::plot({current_path(i,0), current_path(i,0) + dir.x()}, {current_path(i,1), current_path(i,1) + dir.y()}, "k-");

    plt::pause(ANIMATION_SPEED_MS / 1000.0);
}
