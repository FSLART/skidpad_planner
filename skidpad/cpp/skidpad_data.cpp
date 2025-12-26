#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>

// Hardcoded skidpad path
// Reads skidpad-path_raw-values.csv returns (x,y) line by line
int main() {
    std::string filename = "skidpad-path_raw-values.csv";
    std::ifstream file(filename);

    // error handling
    if (!file.is_open()) {
        std::cerr << "Failed to open " << filename << std::endl;
        return 1;
    }

    std::vector<std::pair<double, double>> skidpad_path;
    std::string line;

    // Read CSV line by line
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

    // Print the path to verify
    for (const auto& p : skidpad_path) {
        std::cout << p.first << ", " << p.second << std::endl;
    }

    return 0;
}
