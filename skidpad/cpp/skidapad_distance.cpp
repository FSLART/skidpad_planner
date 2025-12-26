#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <cmath>

// double pos_x = 39.9245262000; 
// double pos_y = -0.0044054606;

// Function to compute Euclidean distance
double distance(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::sqrt(dx*dx + dy*dy);
}

// Returns the next N points ahead of the vehicle
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

    // Read skidpad path from CSV
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

    // Read vehicle position as a single input: x,y
    std::cout << "Enter vehicle position (x,y): ";
    std::string input;
    std::getline(std::cin, input);

    double vehicle_x = 0.0, vehicle_y = 0.0;
    std::stringstream ss(input);
    std::string x_str, y_str;
    if (std::getline(ss, x_str, ',') && std::getline(ss, y_str, ',')) {
        vehicle_x = std::stod(x_str);
        vehicle_y = std::stod(y_str);
    } else {
        std::cerr << "Invalid input format. Use x,y" << std::endl;
        return 1;
    }

    // Get next 40 points ahead
    auto points_ahead = getPointsAhead(skidpad_path, vehicle_x, vehicle_y, 40);

    // Print next points
    std::cout << "Next points ahead:\n";
    for (const auto& p : points_ahead) {
        std::cout << p.first << ", " << p.second << std::endl;
    }

    return 0;
}
