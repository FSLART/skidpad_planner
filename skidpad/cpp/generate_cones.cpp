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
            file << cones[type](i, 0) << ","
                 << cones[type](i, 1) << ","
                 << colors[type] << "\n";
        }
    }

    file.close();
    std::cout << "Saved cones to " << filename << std::endl;
}

// ------------------------------------------------------------
// Generate correct Formula Student skidpad
// ------------------------------------------------------------
std::vector<Eigen::MatrixXd> generate_skidpad(int cones_per_circle)
{
    // ---- FS RULEBOOK CONSTANTS ----
    constexpr double CENTER_SPACING     = 18.0;   // m
    constexpr double CENTERLINE_RADIUS  = 7.625;  // m
    constexpr double TRACK_WIDTH        = 3.0;    // m

    constexpr double R_OUTER = CENTERLINE_RADIUS + TRACK_WIDTH / 2.0; // 9.125
    constexpr double R_INNER = CENTERLINE_RADIUS - TRACK_WIDTH / 2.0; // 6.125

    constexpr double PI = 3.14159265358979323846;
    constexpr double TWO_PI = 2.0 * PI;

    // ---- Circle centers ----
    Eigen::Vector2d left_center (-CENTER_SPACING / 2.0, 0.0);
    Eigen::Vector2d right_center( CENTER_SPACING / 2.0, 0.0);

    // ---- Cone containers ----
    std::vector<Eigen::MatrixXd> cones(4);

    cones[0] = Eigen::MatrixXd(2 * cones_per_circle, 2); // blue
    cones[1] = Eigen::MatrixXd(2 * cones_per_circle, 2); // yellow
    cones[2] = Eigen::MatrixXd(4, 2);                    // orange small
    cones[3] = Eigen::MatrixXd(2, 2);                    // orange big

    // --------------------------------------------------------
    // LEFT circle (CCW)
    // --------------------------------------------------------
    for (int i = 0; i < cones_per_circle; ++i) {
        double t = TWO_PI * i / cones_per_circle;
        Eigen::Vector2d dir(std::cos(t), std::sin(t));

        cones[0].row(i) = left_center + R_OUTER * dir;
        cones[1].row(i) = left_center + R_INNER * dir;
    }

    // --------------------------------------------------------
    // RIGHT circle (CW)
    // --------------------------------------------------------
    for (int i = 0; i < cones_per_circle; ++i) {
        double t = TWO_PI * i / cones_per_circle;
        Eigen::Vector2d dir(std::cos(-t), std::sin(-t));

        cones[0].row(i + cones_per_circle) = right_center + R_OUTER * dir;
        cones[1].row(i + cones_per_circle) = right_center + R_INNER * dir;
    }

    // --------------------------------------------------------
    // Small orange cones (entry/exit gates)
    // --------------------------------------------------------
    cones[2] <<
        -CENTER_SPACING / 2.0,  TRACK_WIDTH / 2.0,
        -CENTER_SPACING / 2.0, -TRACK_WIDTH / 2.0,
         CENTER_SPACING / 2.0,  TRACK_WIDTH / 2.0,
         CENTER_SPACING / 2.0, -TRACK_WIDTH / 2.0;

    // --------------------------------------------------------
    // Big orange cones (center reference)
    // --------------------------------------------------------
    cones[3] <<
        0.0,  CENTERLINE_RADIUS,
        0.0, -CENTERLINE_RADIUS;

    return cones;
}

// ------------------------------------------------------------
// MAIN
// ------------------------------------------------------------
int main()
{
    int cones_per_circle = 120; // smoothness

    auto cones = generate_skidpad(cones_per_circle);

    std::vector<std::string> colors = {
        "blue",
        "yellow",
        "orange_small",
        "orange_big"
    };

    save_cones_to_csv(cones, colors, "cones.csv");

    return 0;
}