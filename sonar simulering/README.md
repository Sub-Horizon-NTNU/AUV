# Ping360 numerical sonar simulation

Step 1 is a standalone C++17 acoustic ray tracer for a stationary sonar in a rectangular pool. It produces inspectable echo paths, return times and relative range profiles. The calculation library has no graphics or third-party dependencies and can be reused by the planned raylib application.

The model covers **reciprocal backscatter paths**, not every possible multipath return. Pool dimensions, pose and acoustic coefficients in the example are illustrative; actual pool specifications and calibration are not yet available. There is no AUV dynamics, IMU (inertial measurement unit), DVL (Doppler velocity log), sensor fusion or GUI in this step.

## Build and run

Open this folder, `sonar simulering`, as a CMake project in CLion. Select the bundled MinGW toolchain and the `sonar_sim` target. Set its working directory to this folder and program arguments to:

```text
--config examples/pool.cfg --angle-grad 0 --output output/ping-001
```

For the tested Windows installation, start PowerShell in this folder and expose CLion's bundled tools and GCC runtime DLLs:

```powershell
$clionRoot = 'C:\Program Files\JetBrains\CLion 2024.3.2'
$env:PATH = "$clionRoot\bin\mingw\bin;$clionRoot\bin\cmake\win\x64\bin;$clionRoot\bin\ninja\win\x64;$env:PATH"
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build
ctest --test-dir build --output-on-failure
.\build\sonar_sim.exe --config examples/pool.cfg --angle-grad 0 --output output/ping-001
.\build\sonar_sim.exe --config examples/pool.cfg --scan --output output/scan-001
```

The same CMake commands work with CMake 3.20+, a C++17 compiler and Ninja on `PATH`; on Unix-like systems invoke `./build/sonar_sim`. Only the Windows toolchain listed below has been verified. `--scan` calculates all 400 motor angles at one fixed pose; it does not simulate scanning time or motion.

Run `sonar_sim --help` for options. Omit `--config` to use the same toy defaults as the example. Select a fresh output subdirectory for each run: existing `echoes.csv`, `paths.csv` or `profiles.csv` files cause an error rather than an overwrite. CSV writing is not atomic as a set; an I/O failure can leave partial files. Inspect those files and rerun with a fresh directory.

If Windows cannot load GCC runtime DLLs, use CLion's toolchain environment or the `PATH` setup above. Configuration and CLI errors exit with code 1; check the reported file, line or option before rerunning.

## Configuration and coordinates

Edit [examples/pool.cfg](examples/pool.cfg). The format is `key=value`, with `#` comments. Omitted keys retain defaults; unknown or duplicate keys, trailing units and invalid/non-finite values are rejected. Change the sonar position when changing dimensions so it remains strictly inside the pool.

| Settings | Meaning and main bounds |
| --- | --- |
| `pool_length_m`, `pool_width_m`, `pool_depth_m` | Box extents in metres; each 0.01–10000. Default 10 × 5 × 3 m. |
| `sonar_x_m`, `sonar_y_m`, `sonar_z_m` | Position in the pool frame; default (5, 2.5, 1.5) m. |
| `sonar_roll_deg`, `sonar_pitch_deg`, `sonar_yaw_deg` | Local-to-pool rotation `Rz(yaw) Ry(pitch) Rx(roll)`; each −360–360 degrees. |
| `sound_speed_m_s`, `max_range_m` | Homogeneous sound speed and apparent range limit; default 1480 m/s and 20 m. Range limit 0.75–50 m. |
| `transmit_duration_us` | Rectangular pulse duration, 1–1000 microseconds; default 20. |
| `wall_reflection`, `floor_reflection`, `surface_reflection` | Reflected **energy** fractions in [0,1]; defaults 0.8, 0.7, 0.95. |
| `backscatter_fraction`, `absorption_per_m` | Backscatter share in [0,1] and energy attenuation coefficient in 1/m; defaults 0.1 and 0.01. |
| `horizontal_rays`, `vertical_rays` | Fan quadrature counts, each 1–101; product at most 1024. Defaults 3 × 11. |
| `max_outward_hits`, `range_samples` | At most 1–16 outward contacts per ray; 200–1200 range bins. Defaults 4 and 600. |

The pool frame is right-handed: x/y are horizontal, z points up, the floor is z=0 and water surface is z=`pool_depth_m`. Local +x is the unrotated beam axis; positive head angle rotates toward local +y. CLI angles are integer gradians 0–399, with 1 gradian = 0.9 degrees. Launch azimuth/elevation fields are in the local sonar frame; path positions are in the pool frame. Positive pitch follows the stated right-handed rotation, pointing local +x toward negative pool z at +90 degrees.

The nominal fan is 2° horizontally × 25° vertically, sampled at cell midpoints with normalized cosine-of-elevation solid-angle weights. It is a uniform beam approximation. All supported bounds and defaults are defined in [the API](include/sonar/simulator.hpp) and `validate()` in [simulator.cpp](src/simulator.cpp).

## Calculation

Each ray intersects the nearest box plane. At each outward contact it contributes a backscatter echo that retraces the entire outward path. The continuing ray is reflected using `d_next = d - 2(d·n)n`, for unit plane normal `n`.

For the kth outward contact at accumulated distance `s` metres:

```text
total_path_m = 2s
travel_time_s = 2s / sound_speed_m_s
apparent_range_m = s
interaction_count = 2k - 1
```

The first contact is labelled direct; later contacts are labelled multipath. These are simulator ground-truth labels, not a classifier for measured sonar data. Complete paths contain the transmitter, outward contacts, reversed earlier contacts and receiver at the transmitter position.

The deliberately uncalibrated relative echo energy is:

```text
I = w P² (R B) μ² [1 m / max(1 m, s)]⁴ exp(-α · 2s)
```

Here `w` is normalized ray weight, `R` the terminal surface energy reflection coefficient, `B` the backscatter fraction, `μ` the absolute incidence cosine and `α`=`absorption_per_m`. `P` is the product of prior specular energy fractions `R(1-B)`; squaring accounts for encountering those surfaces outward and on return. `RB` supplies terminal backscatter while `R(1-B)` continues specularly. The 1 m spreading clamp avoids a near-range singularity and is an arbitrary model reference, not a calibration.

At simultaneous edge/corner contacts, all tied planes are recorded and reflected. The lowest numbered face supplies terminal backscatter and incidence; continuation applies each tied plane's coefficient. `interaction_count` counts contact locations, not individual tied planes. Edge diffraction is absent.

The receiver window is `[0.75, max_range_m)` in apparent metres. Echoes arriving below 0.75 m remain in geometry metadata but contribute no profile energy, even if their pulse tail would cross the boundary. Arrivals exactly at maximum range are metadata only; longer paths are not traced. A causal rectangular pulse of apparent width `c × duration / 2` distributes energy by overlap with uniform range bins. Tails beyond the maximum range are clipped without renormalizing.

## Results and API

Each run writes these CSV files:

| File | Contents |
| --- | --- |
| `echoes.csv` | Per-ping echo/ray IDs, angle, contact counts, direct/multipath label, full length, return time, apparent range, relative intensity and receive-window flag. |
| `paths.csv` | Ordered vertices (`vertex_id`) for every echo, with pool-frame metre coordinates and surface names/bit masks. Join to echoes by `(ping_id, echo_id)`. |
| `profiles.csv` | Range-bin centres, separate direct/multipath integrated relative energies and their sum, joined by `ping_id`. |

IDs restart within each run; `echo_id` is local to a ping. The surface bits are x-min=1, x-max=2, y-min=4, y-max=8, floor=16, water surface=32; zero denotes the sonar. Values are floating-point relative energies, not calibrated pressure or Ping360 hardware byte strengths. Preserve the configuration and invocation alongside exported results when comparing runs.

Use `sonar::simulate_ping(Config, angle_grad)` from [simulator.hpp](include/sonar/simulator.hpp) to obtain `Ping`, `Echo`, `PathVertex` and `RangeBin` values directly. It performs no file or graphics I/O and throws `std::invalid_argument` for invalid input. `sonar_core` is the numerical target; `sonar_io` owns configuration/CSV and `sonar_sim` owns CLI interaction.

## Verification and limits

On 2026-11-24, implementation commit `9c5c44c95c9501bacfbb26b6d166e85afc3627a7` built without warnings using Windows, CLion 2024.3.2 bundled GCC 13.1.0, CMake 3.30.5 and Ninja (Debug). All 3 CTest entries passed: seven numerical groups, CLI help and invalid-angle rejection. Independent inspection of a default 400-angle scan checked 50,852 closed echo paths and 240,000 bins for box bounds, path lengths, return times and finite nonnegative channel energies. Reusing an output directory was rejected without changing the existing echo file.

Formatting was checked on all six handwritten C++ files with CLion's bundled clang-format 20.0.0git (`fd2c1bf18bb22baa1ad38834dce4947d891f67d1`). From this folder:

```powershell
$formatTool = "$clionRoot\bin\clang\win\x64\bin\clang-format.exe"
Get-ChildItem include,src,tests -Recurse -File -Include *.hpp,*.cpp | ForEach-Object { & $formatTool --dry-run --Werror $_.FullName }
```

These are numerical/software checks, not pool or hardware validation. The model omits asymmetric multipath, enumeration of purely specular closed paths, interference, diffraction, side lobes, detailed surface roughness and electronics. It does not solve a frequency-dependent wave field at the Ping360's nominal 750 kHz or emulate its wire protocol. Real sonar classification, scan motion and acoustic calibration remain unverified.

Step 2 is the planned raylib visualization. A configuration panel and synthetic IMU/DVL data are later work; advancement is controlled by the user.

## Sources

- [Blue Robotics Ping360 specifications](https://bluerobotics.com/store/sonars/imaging-sonars/ping360-sonar-r1-rp/): nominal beam widths, minimum/maximum range and motor angular resolution.
- [Ping360 protocol](https://docs.bluerobotics.com/ping-protocol/pingmessage-ping360/): angle and sampled return-strength interface; this simulator exports its own data model.
- [Cerqueira et al., ray-tracing sonar simulation](https://arxiv.org/abs/2001.03539): research background, not a claim that this implementation reproduces that simulator.
- [AUV wiki](https://github.com/Sub-Horizon-NTNU/AUV/wiki/Sonar-simulering): project status and numerical verification.
