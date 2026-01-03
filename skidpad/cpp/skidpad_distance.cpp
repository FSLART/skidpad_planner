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

double distanceToNearestCone(
    double x,
    double y,
    const std::vector<Cone>& cones,
    const std::string& color
) {
    double min_d = 1e9;
    bool found = false;

    for (const auto& c : cones) {
        if (c.color != color) continue;

        double d = distance(x, y, c.x, c.y);
        if (d < min_d) {
            min_d = d;
            found = true;
        }
    }

    return found ? min_d : -1.0;  // -1 = no cone found
}

void saveSamplesToCSV(
    const std::vector<SamplePoint>& samples,
    const std::vector<double>& d_left,
    const std::vector<double>& d_right,
    const std::string& filename
) {
    std::ofstream out(filename);
    if (!out.is_open()) {
        std::cerr << "Failed to write " << filename << "\n";
        return;
    }

    out << "s,x,y,curvature,d_left,d_right\n";

    for (size_t i = 0; i < samples.size(); ++i) {
        out << samples[i].s << ","
            << samples[i].x << ","
            << samples[i].y << ","
            << samples[i].curvature << ","
            << d_left[i] << ","
            << d_right[i] << "\n";
    }

    out.close();
    std::cout << "Saved sampled path to " << filename << "\n";
}


int main() {
    // ---------- Load cones ----------
    auto cones = loadCones("cones.csv");
    std::cout << "Loaded " << cones.size() << " cones\n";

    // ---------- Load vehicle path ----------
    std::ifstream file("car_pos.csv");
    if (!file.is_open()) {
        std::cerr << "Failed to open car_pos.csv\n";
        return 1;
    }

    std::vector<PathPoint> path;
    std::string line;
    std::getline(file, line); // header

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string epoch, h, x, y;

        std::getline(ss, epoch, ',');
        std::getline(ss, h, ',');
        std::getline(ss, x, ',');
        std::getline(ss, y, ',');

        path.push_back({
            std::stod(h),
            std::stod(x),
            std::stod(y)
        });
    }
    file.close();

    if (path.size() < 5) {
        std::cerr << "Path too short\n";
        return 1;
    }

    // ---------- User input ----------
    std::cout << "Enter vehicle state (heading,x,y): ";
    std::getline(std::cin, line);

    std::stringstream ss(line);
    double heading, vx, vy;
    std::string tmp;

    std::getline(ss, tmp, ','); heading = std::stod(tmp);
    std::getline(ss, tmp, ','); vx = std::stod(tmp);
    std::getline(ss, tmp, ','); vy = std::stod(tmp);

    // ---------- Find closest index ----------
    size_t closest = 0;
    double min_d = distance(vx, vy, path[0].x, path[0].y);

    for (size_t i = 1; i < path.size(); ++i) {
        double d = distance(vx, vy, path[i].x, path[i].y);
        if (d < min_d) {
            min_d = d;
            closest = i;
        }
    }

    // ---------- Sample path ----------
    auto s = computeArcLength(path);
    auto samples = sampleEquidistant(path, s, closest, 40, 0.2);

    // ---------- Cone distances ----------
    std::vector<double> d_left, d_right;

    for (const auto& p : samples) {
        d_left.push_back(
            distanceToNearestCone(p.x, p.y, cones, "blue")
        );
        d_right.push_back(
            distanceToNearestCone(p.x, p.y, cones, "yellow")
        );
    }

    // ---------- Save ----------
    saveSamplesToCSV(samples, d_left, d_right, "sampled_path.csv");

    // ---------- Console preview ----------
    std::cout << "\n[s, x, y, k, d_left, d_right]\n";
    for (size_t i = 0; i < samples.size(); ++i) {
        std::cout << samples[i].s << ", "
                  << samples[i].x << ", "
                  << samples[i].y << ", "
                  << samples[i].curvature << ", "
                  << d_left[i] << ", "
                  << d_right[i] << "\n";
    }

    return 0;
}