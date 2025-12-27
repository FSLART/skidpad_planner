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

// Euclidean distance (x,y only)
double distance(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

std::vector<PathPoint> getPointsAhead(
    const std::vector<PathPoint>& path,
    double vx, double vy,
    int N = 40
) {
    if (path.empty()) return {};

    size_t closest_idx = 0;
    double min_dist = distance(vx, vy, path[0].x, path[0].y);

    for (size_t i = 1; i < path.size(); ++i) {
        double d = distance(vx, vy, path[i].x, path[i].y);
        if (d < min_dist) {
            min_dist = d;
            closest_idx = i;
        }
    }

    std::vector<PathPoint> points_ahead;
    for (int i = 1; i <= N && (closest_idx + i) < path.size(); ++i) {
        points_ahead.push_back(path[closest_idx + i]);
    }

    return points_ahead;
}

int main() {
    std::ifstream file("simao_skidpad_run.csv");
    if (!file.is_open()) {
        std::cerr << "Failed to open CSV file\n";
        return 1;
    }

    std::vector<PathPoint> skidpad_path;
    std::string line;

    // Skip header
    std::getline(file, line);

    // epoch_time, heading, x, y
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string epoch_str, heading_str, x_str, y_str;

        if (!std::getline(ss, epoch_str, ',')) continue;
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

     // INPUT: heading,x,y
    std::cout << "Enter vehicle state (heading,x,y): ";
    std::string input;
    std::getline(std::cin, input);

    std::stringstream ss(input);
    std::string heading_str, x_str, y_str;

    if (!std::getline(ss, heading_str, ',') ||
        !std::getline(ss, x_str, ',') ||
        !std::getline(ss, y_str, ',')) {
        std::cerr << "Invalid input format. Use heading,x,y\n";
        return 1;
    }

    double vehicle_heading = std::stod(heading_str);
    double vehicle_x = std::stod(x_str);
    double vehicle_y = std::stod(y_str);

    auto points_ahead = getPointsAhead(skidpad_path, vehicle_x, vehicle_y, 40);

    std::cout << "\nNext points ahead (heading, x, y):\n";
    for (const auto& p : points_ahead) {
        std::cout << p.heading << ", " << p.x << ", " << p.y << "\n";
    }

    return 0;
}