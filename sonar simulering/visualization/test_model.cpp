#include "layout.hpp"
#include "model.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <iostream>
#include <stdexcept>

namespace {

void check(bool condition, const char *message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void near(sonar::Vec3 actual, sonar::Vec3 expected, const char *message) {
    check(sonar::length(actual - expected) < 1e-8, message);
}

} // namespace

int main() {
    try {
        viewer::Model model({});
        const auto original_pose = model.config.pose;
        const auto original_revision = model.scan_revision;
        for (const sonar::Pose pose : {original_pose, sonar::Pose{{2, 3, 1}, 20, -35, 90}}) {
            model.config.pose = pose;
            const sonar::Vec3 local{1.2, -0.7, 0.4};
            near(model.pool_to_sonar(pose.position_m + model.sonar_to_pool_direction(local)), local,
                 "aligned frame must invert the complete sonar pose");
            near(model.pool_to_sonar(pose.position_m + model.sonar_to_pool_direction({1, 0, 0})),
                 {1, 0, 0}, "sonar forward must map to local +X (screen up)");
            near(model.pool_to_sonar(pose.position_m + model.sonar_to_pool_direction({0, 1, 0})),
                 {0, 1, 0}, "sonar left must map to local +Y (screen left)");
        }
        model.config.pose = original_pose;
        model.align_with_sonar();
        check(model.projection == viewer::Projection::Top && model.sonar_aligned &&
                  model.scan_revision == original_revision,
              "alignment changes only the view and preserves the scan cache");
        auto config = model.config;
        config.settings.horizontal_rays = 1;
        config.settings.vertical_rays = 1;
        config.pose.yaw_deg = 90;
        check(model.apply(config, 0), "rotated configuration should apply");
        model.select_range(7.5);
        const auto *echo = model.echo();
        check(echo && echo->is_multipath(), "profile selection should find a later echo");
        near(viewer::apparent_position(*echo), {5.0, 10.0, 1.5},
             "apparent position must follow the rotated pool-frame launch direction");
        near(viewer::path_position(*echo, 0.5), {5.0, 0.0, 1.5},
             "halfway marker should reach the backscatter surface");
        near(viewer::path_position(*echo, 1.0), config.pose.position_m,
             "complete marker should return to the sonar");
        const auto *cached_data = model.ping.echoes.data();
        model.tick(0.25);
        model.projection = viewer::Projection::Side;
        check(model.ping.echoes.data() == cached_data, "playback should reuse the cached ping");
        auto invalid = model.config;
        invalid.pose.position_m.z = -1;
        check(!model.apply(invalid, 0), "invalid draft should be rejected");
        check(model.ping.echoes.data() == cached_data && model.config.pose.position_m.z == 1.5,
              "rejected draft must preserve the previous valid ping and configuration");
        model.cycle_echo(-1);
        check(model.echo() != nullptr, "previous echo should remain selectable");
        model.change_angle(-1);
        check(model.ping.angle_grad == 399, "angle should wrap from 0 to 399");
        const auto scan_revision = model.scan_revision;
        const auto *cached_scan = model.scan[0].data();
        const double reference = model.scan_reference;
        double observed_peak = 0;
        for (const auto &profile : model.scan) {
            check(profile.size() == config.settings.range_samples,
                  "all 400 angle profiles are cached");
            for (const auto &bin : profile) {
                observed_peak =
                    std::max(observed_peak,
                             bin.direct_intensity + bin.multipath_intensity + bin.volume_intensity);
            }
        }
        check(reference == observed_peak && reference > 0,
              "global reference uses strongest combined scan bin");
        model.set_angle(100);
        model.gain_db = 5;
        model.tick(0.5);
        check(model.scan_revision == scan_revision && model.scan[0].data() == cached_scan &&
                  model.scan_reference == reference,
              "angle, gain and playback do not rebuild or renormalize the scan");
        model.gain_db = 0;
        check(std::abs(model.display_level(reference * 1e-4) - 0.5) < 1e-10,
              "display uses energy decibels and the shared 80 dB span");
        check(model.select_image(0.375, 0) && model.ping.angle_grad == 0 && model.echo() &&
                  model.echo()->is_multipath(),
              "image click selects matching angle and nearest arrival");
        check(model.select_image(0, 0.2) && model.ping.angle_grad == 100,
              "sonar-local positive y maps to 100 grad regardless of world yaw");
        check(!model.select_image(2, 0), "image clicks outside the circle are ignored");
        model.set_channel(viewer::Channel::Volume);
        model.select_range(5);
        check(model.echo() == nullptr, "a channel with no arrivals must not select a surface echo");
        model.set_channel(viewer::Channel::Combined);
        check(model.echo() && model.accepts(*model.echo()),
              "switching from empty Particles to All must restore an eligible selection");
        model.set_channel(viewer::Channel::Volume);
        check(model.echo() == nullptr, "Particles remains empty when configured count is zero");
        model.set_channel(viewer::Channel::Direct);
        check(model.echo() && model.accepts(*model.echo()) &&
                  model.echo()->kind == sonar::EchoKind::Surface && !model.echo()->is_multipath(),
              "switching from empty Particles to Direct must restore a direct surface selection");
        check(model.scan_revision == scan_revision && model.scan[0].data() == cached_scan,
              "channel recovery must reuse the existing scan cache");
        model.set_channel(viewer::Channel::Combined);
        model.orbit_camera(90, 200);
        check(model.camera_pitch_deg == 80 && model.orbit_deg != 35,
              "camera orbit changes yaw and clamps pitch before its pole");
        model.zoom_camera(1000);
        check(model.camera_zoom == 0.3F, "zoom in is bounded");
        model.zoom_camera(-1000);
        check(model.camera_zoom == 3.5F, "zoom out is bounded");
        model.reset_camera();
        model.show_particle_markers = false;
        check(model.camera_pitch_deg == 32 && model.orbit_deg == 35 && model.camera_zoom == 1 &&
                  model.scan_revision == scan_revision && model.scan[0].data() == cached_scan &&
                  model.config.settings.volume_scatterers == 0,
              "camera and marker visibility preserve physics and the scan cache");
        model.set_angle(399);
        model.progress = 0.37;
        model.playing = false;
        model.set_scan_playing(true);
        model.advance_scan(0.05);
        check(model.ping.angle_grad == 1 && model.scan_playing && model.progress == 0.37 &&
                  !model.playing,
              "360 scan wraps through zero and preserves the independent propagation clock");
        model.advance_scan(1000000);
        check(model.ping.angle_grad == 1 && model.scan_revision == scan_revision &&
                  model.scan[0].data() == cached_scan,
              "large scan time steps are bounded and never rebuild the full scan");
        model.change_angle(1);
        check(!model.scan_playing && model.ping.angle_grad == 2,
              "manual angle changes pause the 360 scan");
        model.set_scan_playing(true);
        model.progress = 0.2;
        model.playing = true;
        model.tick(6);
        check(model.playing && std::abs(model.progress - 0.4) < 1e-10 &&
                  model.scan_revision == scan_revision && model.scan[0].data() == cached_scan,
              "active 360 scanning repeats pulse playback beyond five seconds without rebuilding "
              "the scan");
        model.playing = false;
        const double paused_progress = model.progress;
        const unsigned previous_heading = model.ping.angle_grad;
        model.tick(0.1);
        check(!model.playing && model.progress == paused_progress &&
                  model.ping.angle_grad == (previous_heading + 4) % 400,
              "pausing the pulse preserves its progress while the scan heading continues");
        model.set_scan_playing(false);
        model.progress = 0.2;
        model.playing = true;
        model.loop_playback = false;
        model.tick(6);
        check(!model.playing && model.progress == 1,
              "single-angle pulse playback still stops after one traversal");
        model.set_scan_playing(true);
        model.select_image(0.2, 0);
        check(!model.scan_playing, "clicking the image pauses scanning to inspect an echo");
        bool rejected_time = false;
        try {
            model.advance_scan(-1);
        } catch (const std::invalid_argument &) {
            rejected_time = true;
        }
        check(rejected_time, "negative scan time must be rejected");
        for (const auto &dimensions :
             {std::pair<int, int>{1000, 700}, {1280, 800}, {1600, 900}, {1920, 1080}}) {
            const auto layout = viewer::calculate_layout(dimensions.first, dimensions.second);
            const auto expanded =
                viewer::calculate_layout(dimensions.first, dimensions.second, true);
            for (const auto area :
                 {layout.ray_panel, layout.world, layout.image_panel, layout.image,
                  layout.transport, layout.inspector, layout.configuration}) {
                check(area.width > 0 && area.height > 0 && area.x >= 0 && area.y >= 0 &&
                          area.x + area.width <= static_cast<float>(dimensions.first) &&
                          area.y + area.height <= static_cast<float>(dimensions.second),
                      "native layout must keep every active panel inside the window");
            }
            check(layout.image.width == layout.image.height &&
                      layout.ray_panel.x + layout.ray_panel.width < layout.image_panel.x &&
                      layout.world.height > expanded.world.height && layout.profile.height == 0,
                  "comparison panels do not overlap; image aspect is fixed; closing profile gives "
                  "space to geometry");
        }
        model.all_returns = true;
        model.wavefront_mode = false;
        model.progress = 0.25;
        sonar::Echo short_route;
        short_route.total_path_m = 4;
        short_route.travel_time_s = 4 / model.config.settings.sound_speed_m_s;
        short_route.outbound_path_m = 1;
        short_route.return_path_m = 3;
        short_route.path = {{{0, 0, 0}, 0}, {{1, 0, 0}, 2}, {{0, 0, 0}, 0}};
        short_route.in_receive_window = true;
        short_route.relative_intensity = 1;
        check(viewer::return_fraction(short_route) == 0.25,
              "return phase follows outbound distance rather than half the full route");
        check(model.playback_time_s() == model.progress * 2 * model.config.settings.max_range_m /
                                             model.config.settings.sound_speed_m_s,
              "simultaneous returns share the full receiver window clock");
        check(model.route_fraction(short_route) == 1,
              "a short route is already received while longer routes may still travel");
        model.select(0);
        check(model.progress == 0.25, "inspecting a return does not rewind the shared ping clock");
        const auto saved_ping = model.ping;
        const auto saved_channel = model.channel;
        model.channel = viewer::Channel::Combined;
        model.ping.echoes = {short_route};
        check(model.arrived_count() == 1 && model.received_count() == 1,
              "common time counts only accepted arrivals");
        model.ping.echoes[0].in_receive_window = false;
        check(model.arrived_count() == 0 && model.received_count() == 0,
              "out-of-window geometry is not a received return");
        model.ping = saved_ping;
        model.channel = saved_channel;
        viewer::Model history({});
        history.wavefront_mode = true;
        history.set_scan_playing(true);
        history.tick(1);
        check(history.emitted_fronts.size() == 5,
              "illustrative scan emits distinct fronts on its cadence");
        const auto first_angle = history.emitted_fronts.front().angle_grad;
        const auto first_direction = history.emitted_fronts.front().geometry.rays.front().direction;
        const auto first_origin = history.emitted_fronts.front().geometry.origin_m;
        const double first_age = history.emitted_fronts.front().age_s;
        history.tick(.1);
        check(history.emitted_fronts.front().angle_grad == first_angle &&
                  history.emitted_fronts.front().age_s > first_age,
              "previous fronts age independently without rotating to the current head");
        near(history.emitted_fronts.front().geometry.rays.front().direction, first_direction,
             "emission direction is immutable");
        near(history.emitted_fronts.front().geometry.origin_m, first_origin,
             "emission origin is immutable");
        history.set_scan_playing(false);
        const double before_head_pause_age = history.emitted_fronts.front().age_s;
        const unsigned paused_head = history.ping.angle_grad;
        history.tick(.1);
        check(history.ping.angle_grad == paused_head &&
                  history.emitted_fronts.front().angle_grad == first_angle &&
                  history.emitted_fronts.front().age_s > before_head_pause_age,
              "stopping the head preserves emitted geometry and advancing pulse ages");
        history.playing = false;
        const auto paused_age = history.emitted_fronts.front().age_s;
        const auto paused_count = history.emitted_fronts.size();
        history.tick(2);
        check(history.emitted_fronts.size() == paused_count &&
                  history.emitted_fronts.front().age_s == paused_age,
              "pulse pause freezes ages and suppresses emissions while the head may turn");
        history.playing = true;
        history.tick(10);
        check(history.emitted_fronts.size() <= 32 &&
                  std::all_of(history.emitted_fronts.begin(), history.emitted_fronts.end(),
                              [&](const viewer::EmittedFront &front) {
                                  return front.age_s <= history.display_horizon_s() + 1e-12;
                              }),
              "expired fronts are removed and retained history is bounded");
        history.set_angle(30);
        check(history.emitted_fronts.empty(), "manual heading clears prior front history");
        history.loop_playback = true;
        history.playback_seconds = 5;
        history.progress = .2;
        history.tick(12);
        check(std::abs(history.progress - .6) < 1e-10 && history.playing,
              "auto replay preserves remainder across multiple cycles");
        viewer::Model paused_launch({});
        paused_launch.wavefront_mode = true;
        paused_launch.playing = false;
        paused_launch.progress = .4;
        paused_launch.set_scan_playing(true);
        const auto launch_heading = paused_launch.emitted_fronts.front().angle_grad;
        const auto launch_age = paused_launch.emitted_fronts.front().age_s;
        paused_launch.tick(1);
        check(paused_launch.emitted_fronts.size() == 1 &&
                  paused_launch.emitted_fronts.front().angle_grad == launch_heading &&
                  paused_launch.emitted_fronts.front().age_s == launch_age,
              "starting the rotating head while pulse-paused snapshots the existing front rather "
              "than rotating it");
        viewer::Model partitioned({});
        viewer::Model whole({});
        for (auto *state : {&partitioned, &whole}) {
            state->wavefront_mode = true;
            state->set_scan_playing(true);
        }
        whole.tick(1);
        for (int step = 0; step < 10; ++step)
            partitioned.tick(.1);
        check(whole.emitted_fronts.size() == partitioned.emitted_fronts.size(),
              "front emission count is independent of dt partition");
        for (std::size_t index = 0; index < whole.emitted_fronts.size(); ++index) {
            check(whole.emitted_fronts[index].angle_grad ==
                          partitioned.emitted_fronts[index].angle_grad &&
                      std::abs(whole.emitted_fronts[index].age_s -
                               partitioned.emitted_fronts[index].age_s) < 1e-10,
                  "front launch angles and ages are independent of frame rate");
        }
        const auto unchanged_range = history.config.settings.max_range_m;
        history.simulation_window_ms = 80;
        check(history.display_horizon_s() == .08 &&
                  history.config.settings.max_range_m == unchanged_range,
              "explicit display horizon never changes core receiver range");
        sonar::Echo proxy;
        proxy.total_path_m = 10;
        proxy.path = {{{0, 0, 0}, 0}, {{5, 0, 0}, 2}, {{0, 0, 0}, 0}};
        auto fading = history.config;
        fading.pool.wall_reflection = 0;
        check(viewer::route_transport_energy(fading, proxy, .4) > 0 &&
                  viewer::route_transport_energy(fading, proxy, .6) == 0,
              "transport marker proxy loses all energy at an absorbing contact");
        const auto inference_revision = model.estimate_revision;
        const auto inference_mask = model.estimate.retained_bins[0];
        const auto raw_reference = model.scan_reference;
        model.gain_db = 7;
        model.dynamic_range_db = 90;
        model.set_channel(viewer::Channel::Multipath);
        model.set_estimated_pool(true);
        check(model.estimated_pool && model.channel == viewer::Channel::Combined &&
                  model.gain_db == 7 && model.dynamic_range_db == 90,
              "Estimated pool selects combined input without changing raw gain/span");
        model.change_angle(1);
        check(model.estimate_revision == inference_revision &&
                  model.estimate.retained_bins[0] == inference_mask &&
                  model.scan_reference == raw_reference,
              "heading and display changes reuse the once-per-scan estimate");
        for (unsigned heading = 0; heading < 400; heading += 13) {
            for (std::size_t bin = 0; bin < model.scan[heading].size(); ++bin) {
                const auto &sample = model.scan[heading][bin];
                const double measured =
                    sample.direct_intensity + sample.multipath_intensity + sample.volume_intensity;
                const double shown = model.image_energy(heading, bin);
                check(shown == 0 || shown == measured, "inference display retains observed energy "
                                                       "without synthesizing or rescaling it");
            }
        }
        model.set_channel(viewer::Channel::Direct);
        check(!model.estimated_pool && model.gain_db == 7 && model.dynamic_range_db == 90,
              "selecting a truth-debug channel exits Estimated mode and preserves gain/span");
        const auto before_move_pose = model.config.pose;
        const auto before_move_angle = model.ping.angle_grad;
        const auto before_move_channel = model.channel;
        const auto before_move_revision = model.scan_revision;
        check(model.move_stationary({3, 2, 999}), "valid top-view move must apply");
        near(model.config.pose.position_m, {3, 2, before_move_pose.position_m.z},
             "top-view move preserves depth");
        near(model.draft.pose.position_m, model.config.pose.position_m,
             "move synchronizes configuration draft");
        check(model.scan_revision == before_move_revision + 1 &&
                  model.estimate_revision == model.scan_revision &&
                  model.ping.angle_grad == before_move_angle &&
                  model.channel == before_move_channel &&
                  model.config.pose.roll_deg == before_move_pose.roll_deg &&
                  model.config.pose.pitch_deg == before_move_pose.pitch_deg &&
                  model.config.pose.yaw_deg == before_move_pose.yaw_deg,
              "move rebuilds one stationary scan and preserves angle/channel/orientation");
        const auto moved_revision = model.scan_revision;
        const auto *moved_scan = model.scan[0].data();
        const auto moved_pose = model.config.pose.position_m;
        check(!model.move_stationary({0, 2, 0}) && !model.move_stationary({std::nan(""), 2, 0}),
              "invalid top-view positions must be rejected");
        near(model.config.pose.position_m, moved_pose,
             "invalid move preserves the committed position");
        check(model.scan_revision == moved_revision && model.scan[0].data() == moved_scan,
              "invalid move preserves the previous scan cache");
        config = model.config;
        config.settings.max_range_m = 0.75;
        check(model.apply(config, 0) && model.echo() == nullptr,
              "empty results should be supported");
        model.cycle_echo(1);
        model.select_range(0.5);
        check(model.scan_reference == 0 && model.display_level(1) == 0,
              "empty scan normalization stays finite and dark");
        std::cout << "PASS: viewer cached state, pose-correct ghost, animation, selection, invalid "
                     "draft, empty results\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
