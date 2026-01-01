#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>

// ------------------------------------------------------------
// Save cones to CSV
// ------------------------------------------------------------
void save_cones_to_csv(
    const std::vector<Eigen::MatrixXd>& cones,
    const std::vector<std::string>& colors,
    const std::string& filename
) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open " << filename << std::endl;
        return;
    }

    file << "x,y,color\n";

    for (size_t type = 0; type < cones.size(); ++type) {
        for (int i = 0; i < cones[type].rows(); ++i) {
            file << cones[type](i,0) << ","
                 << cones[type](i,1) << ","
                 << colors[type] << "\n";
        }
    }

    file.close();
    std::cout << "Saved cones to " << filename << std::endl;
}

// ------------------------------------------------------------
// Generate full Formula Student skidpad
// ------------------------------------------------------------
std::vector<Eigen::MatrixXd> generate_skidpad(
    int cones_per_half,
    double center_radius,
    double track_width
) {
    std::vector<Eigen::MatrixXd> cones(4);

    cones[0] = Eigen::MatrixXd(2 * cones_per_half, 2); // blue
    cones[1] = Eigen::MatrixXd(2 * cones_per_half, 2); // yellow
    cones[2] = Eigen::MatrixXd(4, 2);                  // orange small
    cones[3] = Eigen::MatrixXd(2, 2);                  // orange big

    double r_outer = center_radius + track_width / 2.0;
    double r_inner = center_radius - track_width / 2.0;

    Eigen::Vector2d left_center (-center_radius, 0.0);
    Eigen::Vector2d right_center( center_radius, 0.0);

    // ---- Left circle (CCW)
    constexpr double M_PI = 3.14159265358979323846;

    for (int i = 0; i < cones_per_half; ++i) {
        double t = M_PI * i / (cones_per_half - 1);

        cones[0].row(i) = left_center  + r_outer * Eigen::Vector2d(cos(t), sin(t));
        cones[1].row(i) = left_center  + r_inner * Eigen::Vector2d(cos(t), sin(t));
    }

    // ---- Right circle (CW)
    for (int i = 0; i < cones_per_half; ++i) {
        double t = M_PI * i / (cones_per_half - 1);

        cones[0].row(i + cones_per_half) =
            right_center + r_outer * Eigen::Vector2d(cos(M_PI - t), sin(M_PI - t));

        cones[1].row(i + cones_per_half) =
            right_center + r_inner * Eigen::Vector2d(cos(M_PI - t), sin(M_PI - t));
    }

    // ---- Orange cones
    cones[2] << -center_radius,  track_width / 2,
                -center_radius, -track_width / 2,
                 center_radius,  track_width / 2,
                 center_radius, -track_width / 2;

    cones[3] << 0.0,  track_width,
                0.0, -track_width;

    return cones;
}

// ------------------------------------------------------------
// MAIN
// ------------------------------------------------------------
int main() {
    int cones_per_half = 100;
    double radius = 9.0;
    double width  = 3.0;

    auto cones = generate_skidpad(cones_per_half, radius, width);

    std::vector<std::string> colors = {
        "blue", "yellow", "orange_small", "orange_big"
    };

    save_cones_to_csv(cones, colors, "cones.csv");

    return 0;
}