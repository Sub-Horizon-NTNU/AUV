#include "sonar/io.hpp"

#include <charconv>
#include <fstream>
#include <iomanip>
#include <map>
#include <set>
#include <stdexcept>
#include <string>

namespace sonar {
namespace {

std::string trim(const std::string &text) {
    const auto first = text.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return {};
    }
    return text.substr(first, text.find_last_not_of(" \t\r\n") - first + 1);
}

double parse_number(const std::string &text) {
    std::size_t consumed = 0;
    const double result = std::stod(text, &consumed);
    if (consumed != text.size()) {
        throw std::invalid_argument("expected a number without trailing characters");
    }
    return result;
}

unsigned parse_count(const std::string &text) {
    unsigned result = 0;
    const auto parsed = std::from_chars(text.data(), text.data() + text.size(), result);
    if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size()) {
        throw std::invalid_argument("expected a nonnegative integer");
    }
    return result;
}

std::string surface_labels(std::uint8_t mask) {
    if (mask == 0) {
        return "sonar";
    }
    std::string result;
    for (unsigned face = 0; face < 6; ++face) {
        if ((mask & (1U << face)) != 0) {
            if (!result.empty()) {
                result += '|';
            }
            result += surface_name(static_cast<Surface>(face));
        }
    }
    return result;
}

std::ofstream output_file(const std::filesystem::path &path) {
    std::ofstream stream(path);
    stream.exceptions(std::ios::badbit | std::ios::failbit);
    stream << std::setprecision(17);
    return stream;
}

} // namespace

Config read_config(std::istream &input) {
    Config config;
    const std::map<std::string, double *> numbers{
        {"pool_length_m", &config.pool.length_m},
        {"pool_width_m", &config.pool.width_m},
        {"pool_depth_m", &config.pool.depth_m},
        {"wall_reflection", &config.pool.wall_reflection},
        {"floor_reflection", &config.pool.floor_reflection},
        {"surface_reflection", &config.pool.surface_reflection},
        {"sonar_x_m", &config.pose.position_m.x},
        {"sonar_y_m", &config.pose.position_m.y},
        {"sonar_z_m", &config.pose.position_m.z},
        {"sonar_roll_deg", &config.pose.roll_deg},
        {"sonar_pitch_deg", &config.pose.pitch_deg},
        {"sonar_yaw_deg", &config.pose.yaw_deg},
        {"sound_speed_m_s", &config.settings.sound_speed_m_s},
        {"max_range_m", &config.settings.max_range_m},
        {"transmit_duration_us", &config.settings.transmit_duration_us},
        {"absorption_per_m", &config.settings.absorption_per_m},
        {"backscatter_fraction", &config.settings.backscatter_fraction},
    };
    const std::map<std::string, unsigned *> counts{
        {"horizontal_rays", &config.settings.horizontal_rays},
        {"vertical_rays", &config.settings.vertical_rays},
        {"max_outward_hits", &config.settings.max_outward_hits},
        {"range_samples", &config.settings.range_samples},
    };

    std::set<std::string> seen;
    std::string line;
    unsigned line_number = 0;
    while (std::getline(input, line)) {
        ++line_number;
        line = trim(line.substr(0, line.find('#')));
        if (line.empty()) {
            continue;
        }
        const auto separator = line.find('=');
        if (separator == std::string::npos || line.find('=', separator + 1) != std::string::npos) {
            throw std::invalid_argument("config line " + std::to_string(line_number) +
                                        ": expected key=value");
        }
        const std::string key = trim(line.substr(0, separator));
        const std::string value = trim(line.substr(separator + 1));
        try {
            if (!seen.insert(key).second) {
                throw std::invalid_argument("duplicate key");
            }
            if (const auto field = numbers.find(key); field != numbers.end()) {
                *field->second = parse_number(value);
            } else if (const auto count_field = counts.find(key); count_field != counts.end()) {
                *count_field->second = parse_count(value);
            } else {
                throw std::invalid_argument("unknown key");
            }
        } catch (const std::exception &error) {
            throw std::invalid_argument("config line " + std::to_string(line_number) + " (" + key +
                                        "): " + error.what());
        }
    }
    if (input.bad()) {
        throw std::runtime_error("failed while reading configuration");
    }
    validate(config);
    return config;
}

Config read_config(const std::filesystem::path &path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("cannot read config file: " + path.string());
    }
    try {
        return read_config(input);
    } catch (const std::exception &error) {
        throw std::invalid_argument(path.string() + ": " + error.what());
    }
}

void write_csv(const Config &config, unsigned angle_grad, bool full_scan,
               const std::filesystem::path &directory) {
    validate(config);
    if (angle_grad > 399) {
        throw std::invalid_argument("angle_grad must be in [0,399]");
    }
    const auto echo_path = directory / "echoes.csv";
    const auto path_path = directory / "paths.csv";
    const auto profile_path = directory / "profiles.csv";
    for (const auto &path : {echo_path, path_path, profile_path}) {
        if (std::filesystem::exists(path)) {
            throw std::runtime_error("output already exists: " + path.string() +
                                     "; choose a new --output directory");
        }
    }
    std::filesystem::create_directories(directory);
    auto echoes = output_file(echo_path);
    auto paths = output_file(path_path);
    auto profiles = output_file(profile_path);
    echoes << "ping_id,angle_grad,echo_id,ray_id,launch_azimuth_deg,launch_elevation_deg,"
              "outward_hits,interaction_count,is_multipath,total_path_m,travel_time_s,"
              "apparent_range_m,relative_intensity,in_receive_window\n";
    paths << "ping_id,echo_id,vertex_id,x_m,y_m,z_m,surface_mask,surfaces\n";
    profiles << "ping_id,angle_grad,bin_index,center_range_m,direct_intensity,"
                "multipath_intensity,total_intensity\n";

    const unsigned ping_count = full_scan ? 400U : 1U;
    for (unsigned ping_id = 0; ping_id < ping_count; ++ping_id) {
        const Ping ping = simulate_ping(config, full_scan ? ping_id : angle_grad);
        for (std::size_t echo_id = 0; echo_id < ping.echoes.size(); ++echo_id) {
            const auto &echo = ping.echoes[echo_id];
            echoes << ping_id << ',' << ping.angle_grad << ',' << echo_id << ',' << echo.ray_id
                   << ',' << echo.launch_azimuth_deg << ',' << echo.launch_elevation_deg << ','
                   << echo.outward_hits << ',' << echo.interaction_count << ','
                   << echo.is_multipath() << ',' << echo.total_path_m << ',' << echo.travel_time_s
                   << ',' << echo.apparent_range_m << ',' << echo.relative_intensity << ','
                   << echo.in_receive_window << '\n';
            for (std::size_t vertex_id = 0; vertex_id < echo.path.size(); ++vertex_id) {
                const auto &vertex = echo.path[vertex_id];
                paths << ping_id << ',' << echo_id << ',' << vertex_id << ',' << vertex.position_m.x
                      << ',' << vertex.position_m.y << ',' << vertex.position_m.z << ','
                      << static_cast<unsigned>(vertex.surface_mask) << ','
                      << surface_labels(vertex.surface_mask) << '\n';
            }
        }
        for (std::size_t index = 0; index < ping.profile.size(); ++index) {
            const auto &bin = ping.profile[index];
            profiles << ping_id << ',' << ping.angle_grad << ',' << index << ','
                     << bin.center_range_m << ',' << bin.direct_intensity << ','
                     << bin.multipath_intensity << ','
                     << bin.direct_intensity + bin.multipath_intensity << '\n';
        }
    }
    echoes.close();
    paths.close();
    profiles.close();
}

} // namespace sonar
