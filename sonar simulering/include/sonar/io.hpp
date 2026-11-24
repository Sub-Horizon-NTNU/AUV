#pragma once

#include "sonar/simulator.hpp"

#include <filesystem>
#include <iosfwd>

namespace sonar {

/// Overlay key=value settings onto the illustrative defaults, then validate.
/// Unknown/duplicate keys and malformed values throw std::invalid_argument.
Config read_config(std::istream &input);
Config read_config(const std::filesystem::path &path);

/// Streams one or more pings to three CSV files. Refuses to overwrite existing files.
/// I/O errors throw std::runtime_error; previously written files can be partial on failure.
void write_csv(const Config &config, unsigned angle_grad, bool full_scan,
               const std::filesystem::path &directory);

} // namespace sonar
