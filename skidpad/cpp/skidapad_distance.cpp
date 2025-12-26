#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <cmath>

double pos_x = 39.9245262000;
double pos_y = -0.0044054606;


// Function to compute Euclidean distance
double distance(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::sqrt(dx*dx + dy*dy);
}

std::vector<std::pair<double,double>> getPointsAhead(
    const std::vector<std::pair<double,double>>& path,
    double vx, double vy,
    int N = 40
) {
    if (path.empty()) return {};

    // Find the closest point index
    int closest_idx = 0;
    double min_dist = distance(vx, vy, path[0].first, path[0].second);
    for (size_t i = 1; i < path.size(); ++i) {
        double d = distance(vx, vy, path[i].first, path[i].second);
        if (d < min_dist) {
            min_dist = d;
            closest_idx = i;
        }
    }

    // Start from the next point, not the closest
    std::vector<std::pair<double,double>> points_ahead;
    for (int i = 1; i <= N && (closest_idx + i) < path.size(); ++i) {
        points_ahead.push_back(path[closest_idx + i]);
    }

    return points_ahead;
}

int main() {
    std::string filename = "skidpad-path_raw-values.csv";
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open " << filename << std::endl;
        return 1;
    }

    std::vector<std::pair<double, double>> skidpad_path;
    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string x_str, y_str;
        if (!std::getline(ss, x_str, ',')) continue;
        if (!std::getline(ss, y_str, ',')) continue;
        double x = std::stod(x_str);
        double y = std::stod(y_str);
        skidpad_path.emplace_back(x, y);
    }
    file.close();

    // Example vehicle position
    double vehicle_x = pos_x;
    double vehicle_y = pos_y;

    auto points_ahead = getPointsAhead(skidpad_path, vehicle_x, vehicle_y, 40);

    // Print next 40 points
    std::cout << "Next 40 points ahead:\n";
    for (const auto& p : points_ahead) {
        std::cout << p.first << ", " << p.second << std::endl;
    }

    return 0;
}
