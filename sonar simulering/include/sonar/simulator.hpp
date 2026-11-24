#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace sonar {

struct Vec3 {
    double x{};
    double y{};
    double z{};
};

Vec3 operator+(Vec3 a, Vec3 b);
Vec3 operator-(Vec3 a, Vec3 b);
Vec3 operator*(Vec3 a, double scale);
double dot(Vec3 a, Vec3 b);
double length(Vec3 a);
/// Specular reflection. The caller supplies a unit surface normal.
Vec3 reflect(Vec3 direction, Vec3 unit_normal);

enum class Surface : unsigned { XMin, XMax, YMin, YMax, Floor, WaterSurface };
const char *surface_name(Surface surface);

struct Pool {
    double length_m{10.0};
    double width_m{5.0};
    double depth_m{3.0};
    // Energy reflection coefficients. Defaults are illustrative, not material measurements.
    double wall_reflection{0.8};
    double floor_reflection{0.7};
    double surface_reflection{0.95};
};

struct Pose {
    // Right-handed pool frame: x/y horizontal, z up; floor z=0, surface z=depth_m.
    Vec3 position_m{5.0, 2.5, 1.5};
    // Local-to-pool rotation Rz(yaw) Ry(pitch) Rx(roll), right-hand-positive angles.
    double roll_deg{};
    double pitch_deg{};
    double yaw_deg{};
};

enum class SurfaceModel { Legacy, Roughness };
enum class EchoKind { Surface, Volume };

struct Settings {
    double sound_speed_m_s{1480.0};
    double max_range_m{20.0};
    double transmit_duration_us{20.0};
    // Energy loss exp(-absorption_per_m * total_path_m), excluding boundary losses.
    double absorption_per_m{0.01};
    // Legacy: R*B supplies backscatter and R*(1-B) continues specularly.
    // Roughness: B controls only the diffuse return share; coherent return is angle dependent.
    double backscatter_fraction{0.1};
    unsigned horizontal_rays{3};
    unsigned vertical_rays{11};
    unsigned max_outward_hits{4};
    unsigned range_samples{600};
    SurfaceModel surface_model{SurfaceModel::Legacy};
    double frequency_hz{750000.0};
    double surface_roughness_m{0.0002};
    // Sparse single-scattering proxies, not homogeneous water reflectors.
    unsigned volume_scatterers{};
    unsigned volume_seed{42};
    double volume_strength{0.001};
};

struct SurfaceResponse {
    double coherent_share{};
    double specular_return{};
    double diffuse_return{};
    double continuation{};
};

/// Heuristic reflected-energy shares before multiplying by the material energy coefficient R.
/// Roughness uses a Gaussian coherent factor and a fixed 1 degree receiver acceptance lobe.
/// Return + continuation <= 1; unreceived diffuse energy is discarded, not branched.
SurfaceResponse surface_response(const Settings &settings, double incidence_cosine);

struct Config {
    Pool pool;
    Pose pose;
    Settings settings;
};

inline constexpr double horizontal_beam_width_deg = 2.0;
inline constexpr double vertical_beam_width_deg = 25.0;
inline constexpr double minimum_range_m = 0.75;

/// Throws std::invalid_argument for non-finite values or unsupported bounds.
void validate(const Config &config);

struct PathVertex {
    Vec3 position_m;
    // Bit i marks Surface i. Zero marks the transmitter/receiver; ties mark all hit planes.
    std::uint8_t surface_mask{};
    bool volume_scatter{}; // With mask zero, distinguishes a particle from the sonar endpoints.
};

struct Echo {
    unsigned ray_id{};
    unsigned outward_hits{};
    // Counts contact locations along the full return path: 2*outward_hits-1.
    unsigned interaction_count{};
    double launch_azimuth_deg{};
    double launch_elevation_deg{};
    double total_path_m{};
    double travel_time_s{};
    double apparent_range_m{};
    double relative_intensity{};
    bool in_receive_window{};
    // Complete closed polyline: TX, outward contacts, reversed contacts, RX.
    std::vector<PathVertex> path;
    EchoKind kind{EchoKind::Surface};
    unsigned scatterer_id{}; // Meaningful only for Volume echoes; ray_id is then UINT_MAX.

    bool is_multipath() const;
};

struct RangeBin {
    double center_range_m{};
    // Integrated relative energy in this bin; values are not hardware uint8 strengths.
    double direct_intensity{};
    double multipath_intensity{};
    double volume_intensity{};
};

struct Ping {
    unsigned angle_grad{};
    std::vector<Echo> echoes;
    std::vector<RangeBin> profile;
};

/**
 * Deterministic geometric approximation for a stationary monostatic sonar.
 * At every sampled outward hit, backscatter retraces the outward path. Earlier
 * contacts are specular in both directions. Asymmetric and purely specular
 * closed paths, interference, diffraction, side lobes and calibrated amplitudes
 * are NOT modeled. Optional volume echoes are straight single-particle roundtrips.
 * Surface roughness/lobe and particle strength are relative heuristics, not calibrated
 * transport or a frequency-dependent wave solver. All interfaces use SI units except angles.
 * Throws std::invalid_argument for invalid config or angle_grad outside [0,399].
 */
Ping simulate_ping(const Config &config, unsigned angle_grad);

/// Deterministic sparse point cloud inside the pool. MT19937 output is mapped explicitly
/// to open-unit coordinates so the same seed produces the same positions across libraries.
std::vector<Vec3> volume_points(const Config &config);

} // namespace sonar
