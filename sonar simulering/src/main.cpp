#include "sonar/io.hpp"

#include <charconv>
#include <exception>
#include <filesystem>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>

namespace {

void print_help() {
    std::cout << "Ping360 numerical acoustic ray tracer (step 1)\n"
                 "Usage: sonar_sim [--config FILE] [--angle-grad N | --scan] [--output DIR]\n\n"
                 "  --config FILE    key=value configuration; omitted keys use toy defaults\n"
                 "  --angle-grad N   One stationary ping, 0..399 gradians (default 0)\n"
                 "  --scan           All 400 angles at one fixed sonar pose\n"
                 "  --output DIR     New CSV output directory (default ./output)\n"
                 "  --help           Show this help\n\n"
                 "Outputs: echoes.csv, paths.csv, profiles.csv. Existing files are preserved.\n"
                 "Model: reciprocal backscatter paths in a 3D box, relative echo energy.\n"
                 "Defaults are illustrative, not measured pool dimensions or calibration.\n"
                 "No raylib GUI, AUV dynamics, IMU or DVL simulation in this step.\n";
}

unsigned angle_value(const std::string &text) {
    unsigned value = 0;
    const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
    if (result.ec != std::errc{} || result.ptr != text.data() + text.size() || value > 399) {
        throw std::invalid_argument("--angle-grad expects an integer in [0,399]");
    }
    return value;
}

} // namespace

int main(int argc, char **argv) {
    try {
        std::optional<std::filesystem::path> config_path;
        std::filesystem::path output_directory{"output"};
        unsigned angle_grad = 0;
        bool angle_set = false;
        bool output_set = false;
        bool full_scan = false;
        for (int index = 1; index < argc; ++index) {
            const std::string option = argv[index];
            if (option == "--help") {
                print_help();
                return 0;
            }
            if (option == "--scan") {
                if (full_scan || angle_set) {
                    throw std::invalid_argument("choose --scan or --angle-grad once");
                }
                full_scan = true;
                continue;
            }
            if (option != "--config" && option != "--output" && option != "--angle-grad") {
                throw std::invalid_argument("unknown option: " + option);
            }
            if (index + 1 >= argc) {
                throw std::invalid_argument("missing value after " + option);
            }
            const std::string value = argv[++index];
            if (option == "--config") {
                if (config_path) {
                    throw std::invalid_argument("--config may be provided once");
                }
                config_path = value;
            } else if (option == "--output") {
                if (output_set || value.empty()) {
                    throw std::invalid_argument("--output expects one nonempty directory path");
                }
                output_directory = value;
                output_set = true;
            } else {
                if (angle_set || full_scan) {
                    throw std::invalid_argument("choose --scan or --angle-grad once");
                }
                angle_grad = angle_value(value);
                angle_set = true;
            }
        }

        const sonar::Config config =
            config_path ? sonar::read_config(*config_path) : sonar::Config{};
        sonar::write_csv(config, angle_grad, full_scan, output_directory);
        std::cout << "Simulated " << (full_scan ? 400 : 1) << " stationary ping(s).\n"
                  << "Pool: " << config.pool.length_m << " x " << config.pool.width_m << " x "
                  << config.pool.depth_m << " m; sonar: (" << config.pose.position_m.x << ", "
                  << config.pose.position_m.y << ", " << config.pose.position_m.z << ") m.\n"
                  << "Range: " << sonar::minimum_range_m << ".." << config.settings.max_range_m
                  << " m; sound speed: " << config.settings.sound_speed_m_s << " m/s.\n"
                  << "CSV files: " << std::filesystem::absolute(output_directory).string() << '\n'
                  << "Echo energy is relative; reciprocal backscatter paths only.\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "sonar_sim: " << error.what() << "\nUse --help for usage.\n";
        return 1;
    }
}
