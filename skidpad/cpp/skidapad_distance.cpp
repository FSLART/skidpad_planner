#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <cmath>

// Struct to hold path point
struct PathPoint {
    double x;
    double y;
    double heading;
};

// Function to compute Euclidean distance
double distance(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::sqrt(dx*dx + dy*dy);
}

// Returns the next N points ahead of the vehicle
std::vector<PathPoint> getPointsAhead(
    const std::vector<PathPoint>& path,
    double vx, double vy,
    int N = 40
) {
    if (path.empty()) return {};

    // Find closest point index (based on x,y)
    size_t closest_idx = 0;
    double min_dist = distance(vx, vy, path[0].x, path[0].y);

    for (size_t i = 1; i < path.size(); ++i) {
        double d = distance(vx, vy, path[i].x, path[i].y);
        if (d < min_dist) {
            min_dist = d;
            closest_idx = i;
        }
    }

    // Collect next N points, skipping the closest
    std::vector<PathPoint> points_ahead;
    for (int i = 1; i <= N && (closest_idx + i) < path.size(); ++i) {
        points_ahead.push_back(path[closest_idx + i]);
    }

    return points_ahead;
}

int main() {
    std::string filename = "simao_skidpad_run.csv";
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Failed to open " << filename << std::endl;
        return 1;
    }

    std::vector<PathPoint> skidpad_path;
    std::string line;

    // Skip header
    std::getline(file, line);

    // Read CSV: epoch_time,heading,x,y
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string time_str, heading_str, x_str, y_str;

        if (!std::getline(ss, time_str, ',')) continue;
        if (!std::getline(ss, heading_str, ',')) continue;
        if (!std::getline(ss, x_str, ',')) continue;
        if (!std::getline(ss, y_str, ',')) continue;

        PathPoint p;
        p.heading = std::stod(heading_str);
        p.x = std::stod(x_str);
        p.y = std::stod(y_str);

        skidpad_path.push_back(p);
    }
    file.close();

    // Read vehicle input as x,y,heading
    std::cout << "Enter vehicle state (x,y,heading): ";
    std::string input;
    std::getline(std::cin, input);

    std::stringstream ss(input);
    std::string x_str, y_str, heading_str;

    if (!std::getline(ss, x_str, ',') ||
        !std::getline(ss, y_str, ',') ||
        !std::getline(ss, heading_str, ',')) {
        std::cerr << "Invalid input format. Use x,y,heading" << std::endl;
        return 1;
    }

    double vehicle_x = std::stod(x_str);
    double vehicle_y = std::stod(y_str);
    double vehicle_heading = std::stod(heading_str);

    // Get next 40 points ahead
    auto points_ahead = getPointsAhead(skidpad_path, vehicle_x, vehicle_y, 40);

    // Output results
    std::cout << "\nNext points ahead (x, y, heading):\n";
    for (const auto& p : points_ahead) {
        std::cout << p.x << ", " << p.y << ", " << p.heading << std::endl;
    }

    return 0;
}