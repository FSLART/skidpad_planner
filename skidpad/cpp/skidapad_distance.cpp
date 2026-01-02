#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <cmath>

// Struct to hold path point (vehicle -> heading,x,y)
struct PathPoint {
    double heading;
    double x;
    double y;
};

// Struct to hold coin data (cone -> x,y,color)
struct Cone {
    double x;
    double y;
    std::string color;
};

// Struct to hold array data
struct SamplePoint {
    double s;
    double x;
    double y;
    double curvature;
};

// Euclidean distance (x,y only)
double distance(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

std::vector<Cone> loadCones(const std::string& filename) {
    std::ifstream file(filename);
    std::vector<Cone> cones;
    std::string line;

    if (!file.is_open()) {
        std::cerr << "Failed to open cones.csv\n";
        return cones;
    }

    std::getline(file, line); // header

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string x_str, y_str, color;

        if (!std::getline(ss, x_str, ',')) continue;
        if (!std::getline(ss, y_str, ',')) continue;
        if (!std::getline(ss, color, ',')) continue;

        cones.push_back({
            std::stod(x_str),
            std::stod(y_str),
            color
        });
    }

    return cones;
}

std::vector<double> computeArcLength(const std::vector<PathPoint>& path) {
    std::vector<double> s(path.size(), 0.0);

    for (size_t i = 1; i < path.size(); ++i) {
        s[i] = s[i - 1] +
               distance(path[i - 1].x, path[i - 1].y,
                        path[i].x, path[i].y);
    }
    return s;
}

double curvature(
    const PathPoint& p1,
    const PathPoint& p2,
    const PathPoint& p3
) {
    double a = distance(p1.x, p1.y, p2.x, p2.y);
    double b = distance(p2.x, p2.y, p3.x, p3.y);
    double c = distance(p3.x, p3.y, p1.x, p1.y);

    double area = std::abs(
        (p2.x - p1.x) * (p3.y - p1.y) -
        (p2.y - p1.y) * (p3.x - p1.x)
    ) * 0.5;

    if (area < 1e-6) return 0.0;

    double R = (a * b * c) / (4.0 * area);
    return 1.0 / R;
}

std::vector<SamplePoint> sampleEquidistant(
    const std::vector<PathPoint>& path,
    const std::vector<double>& s,
    size_t start_idx,
    int N = 40,
    double ds = 0.2   // meters
) {
    std::vector<SamplePoint> samples;

    double s_start = s[start_idx];
    size_t idx = start_idx;

    for (int i = 1; i <= N; ++i) {
        double target_s = s_start + i * ds;

        while (idx + 1 < s.size() && s[idx + 1] < target_s)
            idx++;

        if (idx + 2 >= path.size()) break;

        double k = curvature(path[idx], path[idx + 1], path[idx + 2]);

        samples.push_back({
            target_s - s_start,
            path[idx].x,
            path[idx].y,
            k
        });
    }

    return samples;
}


int main() {
    // -----------------------------
    // Load vehicle path
    // -----------------------------
    std::ifstream file("car_pos.csv");
    if (!file.is_open()) {
        std::cerr << "Failed to open car_pos.csv\n";
        return 1;
    }

    std::vector<PathPoint> skidpad_path;
    std::string line;

    std::getline(file, line); // skip header

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string epoch_str, heading_str, x_str, y_str;

        if (!std::getline(ss, epoch_str, ',')) continue;
        if (!std::getline(ss, heading_str, ',')) continue;
        if (!std::getline(ss, x_str, ',')) continue;
        if (!std::getline(ss, y_str, ',')) continue;

        skidpad_path.push_back({
            std::stod(heading_str),
            std::stod(x_str),
            std::stod(y_str)
        });
    }
    file.close();

    if (skidpad_path.size() < 3) {
        std::cerr << "Not enough path points\n";
        return 1;
    }

    // -----------------------------
    // Load cones (for later use)
    // -----------------------------
    auto cones = loadCones("cones.csv");
    std::cout << "Loaded " << cones.size() << " cones\n";

    // -----------------------------
    // Read vehicle state
    // -----------------------------
    std::cout << "Enter vehicle state (heading,x,y): ";
    std::string input;
    std::getline(std::cin, input);

    std::stringstream ss(input);
    std::string heading_str, x_str, y_str;

    if (!std::getline(ss, heading_str, ',') ||
        !std::getline(ss, x_str, ',') ||
        !std::getline(ss, y_str, ',')) {
        std::cerr << "Invalid input format\n";
        return 1;
    }

    double vehicle_heading = std::stod(heading_str); // unused (for now)
    double vehicle_x = std::stod(x_str);
    double vehicle_y = std::stod(y_str);

    // -----------------------------
    // Find closest path index
    // -----------------------------
    size_t closest_idx = 0;
    double min_dist = distance(
        vehicle_x, vehicle_y,
        skidpad_path[0].x, skidpad_path[0].y
    );

    for (size_t i = 1; i < skidpad_path.size(); ++i) {
        double d = distance(
            vehicle_x, vehicle_y,
            skidpad_path[i].x, skidpad_path[i].y
        );
        if (d < min_dist) {
            min_dist = d;
            closest_idx = i;
        }
    }

    // -----------------------------
    // Compute arc length
    // -----------------------------
    auto arc_length = computeArcLength(skidpad_path);

    // -----------------------------
    // Sample equidistant points
    // -----------------------------
    auto samples = sampleEquidistant(
        skidpad_path,
        arc_length,
        closest_idx,
        40,
        0.2   // spacing in meters
    );

    // -----------------------------
    // Output result
    // -----------------------------
    std::cout << "\n[distance, x, y, curvature]\n";
    for (const auto& p : samples) {
        std::cout << p.s << ", "
                  << p.x << ", "
                  << p.y << ", "
                  << p.curvature << "\n";
    }

    return 0;
}