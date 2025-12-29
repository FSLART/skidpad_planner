#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Dense>

// Function to save cones to CSV
void save_cones_to_csv(const std::vector<Eigen::MatrixXd>& cones_by_type,
                       const std::vector<std::string>& cone_colors,
                       const std::string& filename)
{
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Cannot open file " << filename << std::endl;
        return;
    }

    // CSV header
    file << "x,y,color\n";

    for (size_t type = 0; type < cones_by_type.size(); ++type) {
        const Eigen::MatrixXd& cones = cones_by_type[type];
        std::string color = (type < cone_colors.size()) ? cone_colors[type] : "unknown";

        for (int i = 0; i < cones.rows(); ++i) {
            file << cones(i, 0) << "," << cones(i, 1) << "," << color << "\n";
        }
    }

    file.close();
    std::cout << "Cones saved to " << filename << std::endl;
}

int main()
{
    // Example: 3 types of cones
    std::vector<Eigen::MatrixXd> cones_by_type(3);

    // Type 0 cones (blue)
    cones_by_type[0] = Eigen::MatrixXd(3, 2);
    cones_by_type[0] << 0.0, 0.0,
                         1.0, 0.5,
                         2.0, 0.2;

    // Type 1 cones (yellow)
    cones_by_type[1] = Eigen::MatrixXd(2, 2);
    cones_by_type[1] << 0.5, 1.0,
                         1.5, 1.2;

    // Type 2 cones (orange)
    cones_by_type[2] = Eigen::MatrixXd(2, 2);
    cones_by_type[2] << 0.0, 2.0,
                         1.0, 2.5;

    std::vector<std::string> cone_colors = {"blue", "yellow", "orange"};

    // Save to CSV
    save_cones_to_csv(cones_by_type, cone_colors, "cones.csv");

    return 0;
}
