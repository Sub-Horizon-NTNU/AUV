# Ping360 numerical sonar simulation

Step 1 is a standalone C++17 acoustic ray tracer for a stationary sonar in a rectangular pool. It produces inspectable echo paths, return times and relative range profiles. Step 2 adds an optional raylib viewer for exploring those calculations. The calculation library retains no graphics or third-party dependencies.

The model covers **reciprocal backscatter paths**, not every possible multipath return. Pool dimensions, pose and acoustic coefficients in the example are illustrative; actual pool specifications and calibration are not yet available. AUV dynamics, IMU (inertial measurement unit), DVL (Doppler velocity log) and sensor fusion are outside the implemented scope.

## Build and run

### VS Code on Windows

Open [Sonar.code-workspace](../Sonar.code-workspace) from the repository root. The prepared machine already has VS Code, the recommended Microsoft C/C++ and CMake Tools extensions, and the required compiler/debugger tools. If prompted about Workspace Trust, trust the workspace only if the project is trusted.

For a fresh checkout, run Setup once from the **repository root** in Windows PowerShell, then open or reopen the workspace:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File '.\sonar simulering\scripts\dev.ps1' -Action Setup
```

Setup discovers GCC, CMake, Ninja, CTest and GDB on `PATH` or in an installed CLion bundle. It generates local `.vscode/settings.json` and `launch.json` with the discovered paths. It does not download tools; install missing prerequisites and rerun Setup if discovery fails. The current machine reuses CLion's installed tools without requiring CLion IDE setup.

Use **Ctrl+Shift+B** to build. For other actions, open the Command Palette, choose **Tasks: Run Task**, then select:

| Task | Result |
| --- | --- |
| `Sonar: Visualize` | Build the optional raylib viewer and open the interactive window. |
| `Sonar: Setup` | Generate or refresh the managed local editor/debug configuration. |
| `Sonar: Configure` | Configure the separate Debug build in `build-vscode`. |
| `Sonar: Build` | Configure and build the simulator and tests; default build task. |
| `Sonar: Test` | Build and run CTest. |
| `Sonar: Run` | Build and calculate one ping with `examples/pool.cfg`. |
| `Sonar: Scan` | Build and calculate all 400 stationary angles. |
| `Sonar: Prepare debug` | F5 pre-launch task: build and preserve previous debug output. |

**F5** uses the generated `Sonar: Debug simulator` launch configuration and GDB. Its pre-launch task moves existing `output/debug-current` into a unique directory under `output/debug-history`, preserving the files before the next launch. New debug results go to `output/debug-current`. Run and Scan automatically choose fresh `output/ping-<timestamp>-<id>` or `output/scan-<timestamp>-<id>` directories, printed in the terminal.

The same helper can be used from the repository root without editor shortcuts:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File '.\sonar simulering\scripts\dev.ps1' -Action Build
powershell.exe -NoProfile -ExecutionPolicy Bypass -File '.\sonar simulering\scripts\dev.ps1' -Action Test
powershell.exe -NoProfile -ExecutionPolicy Bypass -File '.\sonar simulering\scripts\dev.ps1' -Action Run
```

Setup is repeatable for files it generated and keeps one named simulator debug configuration. An existing settings/launch file without the script's ownership header is preserved and causes Setup to stop. Back up that file and remove it from the generated destination before retrying; keep any needed custom settings in the backup for deliberate reapplication. Local settings, launch configuration, build and output are ignored by Git. Rerun Setup after moving the checkout or changing tools. The process-specific execution-policy option and tool `PATH` setup do not change global Windows policy or environment settings.

### CLion or manual CMake

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

Run `sonar_sim --help` for options. Without `--config`, API defaults use legacy surfaces and no particles. The supplied example now enables roughness (0.15 mm RMS at 750 kHz) and 1024 particle proxies with strength 0.005 and seed 42. Select a fresh output subdirectory for each run: existing `echoes.csv`, `paths.csv` or `profiles.csv` files cause an error rather than an overwrite. CSV writing is not atomic as a set; an I/O failure can leave partial files. Inspect those files and rerun with a fresh directory.

If Windows cannot load GCC runtime DLLs, use CLion's toolchain environment or the `PATH` setup above. Configuration and CLI errors exit with code 1; check the reported file, line or option before rerunning.

## Interactive raylib viewer (step 2)

In VS Code, choose **Tasks: Run Task → Sonar: Visualize**. Alternatively, from the repository root:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File '.\sonar simulering\scripts\dev.ps1' -Action View
```

The first viewer build requires Git and network access to fetch official raylib 6.2, pinned to commit `02566453880e06bec97f5b8160fa6fe1e98397ca`. Sources stay in the ignored build dependency cache (`build-vscode/_deps`); there is no global raylib installation. The viewer uses OpenGL 3.3 and also needs a C compiler. Windows rendering was checked; other platforms and font appearance remain unverified. Windows uses locally installed Segoe UI when available, otherwise raylib's default font. The native window layout has a 1000 × 700 minimum. Ray targets follow panel size and the sonar texture follows panel/DPI size (384–2048 pixels), without recalculating acoustics on resize.

Compare the selected ping's paths with a cached 400-angle simulated sonar image:

| Control | Action |
| --- | --- |
| `3D`, `Top`, `Side`; drag in 3D; mouse wheel; `Reset` | Change projection, orbit with either mouse button, zoom or reset the camera. |
| Left/Right arrows or angle slider | Change sonar head angle. |
| Up/Down arrows or echo `<` / `>` | Select another model echo. |
| Space or `Pause` / `Play`; `Replay`; progress slider | Pause, replay or scrub the selected sound path. |
| Click the received profile | Select the nearest eligible modeled arrival by apparent range. This is range selection, not measured-echo identification. |
| Click the sonar image; `All`, `Direct`, `Multipath`, `Particles` | Select angle/range and filter contributions. Local +x points up and +y left. Overlapping echoes are not uniquely identified. |
| `Ray paths` / `Wavefront` | Switch between a selected echo's roundtrip and the coarse one-way geometric front. |
| `Scattering >` | Edit legacy/roughness model, RMS roughness and particle enable/count/strength; apply the draft together. |
| Draft `−` / `+`, then `Apply draft` | Change pool dimensions, sonar position, receive range or maximum outward contacts together. Invalid drafts retain the last valid simulation and show an error. |
| `Reload config`; Escape | Reload the supplied config file, or illustrative defaults if none was supplied; close the window. |

Draft edits are not saved to the config file. Orientation and other settings remain configurable through the file. The inspector shows total path, displayed range (`path / 2`), physical return time in milliseconds and contact information.

Cyan marks direct surface energy and the outward pulse; amber marks multipath/return paths and green marks particle contributions. Purple is the **apparent straight-line location**, not a real object, and may lie outside the view. Ray-path playback stretches the selected roundtrip to five seconds. Image and profile share one fixed reference: the maximum **combined** bin over all 400 angles. Gain and span change only display scaling (`10 log10(energy/reference) + gain`), not the physics; default span is 80 dB. These are not calibrated acoustic dB. The image is simulated, not measured sonar data. Configuration changes rebuild the scan/cloud; angle changes recompute only the selected ping, while channel/gain/span changes reuse scan data.

Wavefront mode now uses 32 × 192 rays (6144) in an ideal rectangular mirror geometry: fold `origin + direction*c*t` into the pool. Rays persist for the full one-way `max_range_m/c` window regardless of material energy or echo-contact budget; wall-crossing seam vertices keep neighboring patches connected without false chords across walls. Playback slows that window to five seconds. This is geometry, not pressure, FDTD, interference, diffuse branching or particle/bubble-scattered fronts; it carries no acoustic amplitude model and is separate from the predicted received echoes. The API permits up to 262144 rays.


The default view keeps the two large comparison panels; open `Profile`, `Details` or `Settings` as needed. `Help` (H) explains beam/range conventions and direct versus multipath examples. `Particle markers` hides only background cloud markers in all geometry views; it leaves particle physics, sonar image and selected particle returns intact. `Play 360 scan` rotates the stationary head independently at an illustrative default 40 grad/s (10 seconds/revolution), adjustable with speed buttons. Manual angle/image selection pauses scanning. Pulse playback repeats while scanning; pausing the pulse does not pause heading rotation. These are display clocks, not Ping360 hardware timing.

`SONAR_BUILD_VIEWER` defaults to `OFF`, keeping ordinary CLI builds free of raylib downloads and C-compiler requirements. The helper's `View` action turns it on; other build actions, including `Test`, turn it off and run the three CLI/core tests. To build and test the viewer manually, use the tool environment from the CLion/manual section and run these commands **inside this folder**:

```powershell
cmake -S . -B build-vscode -G Ninja -DCMAKE_BUILD_TYPE=Debug -DSONAR_BUILD_VIEWER=ON
cmake --build build-vscode
ctest --test-dir build-vscode --output-on-failure
.\build-vscode\sonar_viewer.exe --config examples/pool.cfg
```

With the viewer enabled, CTest runs five entries, including viewer-state and geometric-front tests. For optional rendering QA, `sonar_viewer --smoke-test` renders eight frames then exits; `--wavefront` and `--scattering` select startup modes. Add `--screenshot` with an absolute PNG path to capture that run; normal use needs none of these switches.

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
| `surface_model`, `frequency_hz`, `surface_roughness_m` | `legacy` or `roughness`; frequency 500000–1000000 Hz and RMS roughness 0–0.01 m. API defaults: legacy, 750000 Hz, 0.0002 m. |
| `volume_scatterers`, `volume_strength`, `volume_seed` | 0–4096 points, strength 0–1 and unsigned seed. API defaults: 0 points, 0.001 strength, seed 42. |

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

For `surface_model=legacy`, the uncalibrated relative surface echo energy is:

```text
I = w P² (R B) μ² [1 m / max(1 m, s)]⁴ exp(-α · 2s)
```

Here `w` is normalized ray weight, `R` the terminal surface energy reflection coefficient, `B` the backscatter fraction, `μ` the absolute incidence cosine and `α`=`absorption_per_m`. `P` is the product of prior specular energy fractions `R(1-B)`; squaring accounts for encountering those surfaces outward and on return. `RB` supplies terminal backscatter while `R(1-B)` continues specularly. The 1 m spreading clamp avoids a near-range singularity and is an arbitrary model reference, not a calibration.

For `roughness`, wavelength `λ=c/f` is about 1.97 mm at 1480 m/s and 750 kHz. With RMS roughness `σ`, coherent energy share `S=exp(-(4πσμ/λ)²)` and illustrative receiver lobe `L=exp(-0.5[acos(2μ²−1)/1°]²)`, terminal return share is `q=SL+(1−S)Bμ²`; continuation is `S(1−L)`. Multiply each by material energy coefficient `R`. Return plus continuation is bounded by one; unreceived diffuse energy is discarded. In the energy equation replace `Bμ²` by `q` and use the new prior continuation product. Its square remains an effective reciprocal-path heuristic, not exact transport.

The coherent factor follows the energy square of the amplitude factor in [Delft §4.2.3](https://ocw.tudelft.nl/wp-content/uploads/Reader_chapter_4_01.pdf). That source's small-Rayleigh-parameter approximation is not valid across all allowed settings; extension to large roughness/angles and the receiver lobe are explicit heuristics. [Hansen's 2021 UiO introduction, pp. 3–4](https://www.uio.no/studier/emner/matnat/ifi/IN3015/h24/curriculum/sonar_introduction_2021_compressed.pdf) motivates the distinction between smooth specular surfaces, rough diffuse surfaces and volume scattering.

Optional seeded uniform points represent sparse particles/bubbles, not reflection by homogeneous water. Each eligible point contributes one straight TX→point→RX path inside the finite fan, with relative per-particle strength, spreading and absorption. There are no reflected-particle paths, multiple scattering, bubble resonance, refraction or calibrated volume coefficients.

At simultaneous edge/corner contacts, all tied planes are recorded and reflected. The lowest numbered face supplies terminal backscatter and incidence; continuation applies each tied plane's coefficient. `interaction_count` counts contact locations, not individual tied planes. Edge diffraction is absent.

The receiver window is `[0.75, max_range_m)` in apparent metres. Echoes arriving below 0.75 m remain in geometry metadata but contribute no profile energy, even if their pulse tail would cross the boundary. Arrivals exactly at maximum range are metadata only; longer paths are not traced. A causal rectangular pulse of apparent width `c × duration / 2` distributes energy by overlap with uniform range bins. Tails beyond the maximum range are clipped without renormalizing.

## Results and API

Each run writes these CSV files:

| File | Contents |
| --- | --- |
| `echoes.csv` | Per-ping echo/ray IDs, angle, contact counts, direct/multipath label, full length, return time, apparent range, relative intensity and receive-window flag. |
| `paths.csv` | Ordered vertices (`vertex_id`) for every echo, with pool-frame metre coordinates and surface names/bit masks. Join to echoes by `(ping_id, echo_id)`. |
| `profiles.csv` | Range-bin centres, direct/multipath/volume integrated relative energies and their sum, joined by `ping_id`. |

IDs restart within each run; `echo_id` is local to a ping. The surface bits are x-min=1, x-max=2, y-min=4, y-max=8, floor=16, water surface=32; zero denotes the sonar. Values are floating-point relative energies, not calibrated pressure or Ping360 hardware byte strengths. Preserve the configuration and invocation alongside exported results when comparing runs.

CSV additions retain earlier columns: `echo_kind` (`surface`/`volume`) and volume-only `scatterer_id`, path `vertex_kind` (`sonar`/`surface`/`volume`), and `volume_intensity`. A volume echo has `ray_id=UINT_MAX` and `is_multipath=false`; it is **not** a direct surface echo. Zero surface mask can now also denote a particle, distinguished by its vertex kind. `total_intensity` sums all three channels.

Use `sonar::simulate_ping(Config, angle_grad)` from [simulator.hpp](include/sonar/simulator.hpp) to obtain `Ping`, `Echo`, `PathVertex` and `RangeBin` values directly. It performs no file or graphics I/O and throws `std::invalid_argument` for invalid input. `sonar_core` is the numerical target; `sonar_io` owns configuration/CSV and `sonar_sim` owns CLI interaction.

## Verification and limits

On 2026-11-24, implementation commit `9c5c44c95c9501bacfbb26b6d166e85afc3627a7` built without warnings using Windows, CLion 2024.3.2 bundled GCC 13.1.0, CMake 3.30.5 and Ninja (Debug). All 3 CTest entries passed: seven numerical groups, CLI help and invalid-angle rejection. Independent inspection of a default 400-angle scan checked 50,852 closed echo paths and 240,000 bins for box bounds, path lengths, return times and finite nonnegative channel energies. Reusing an output directory was rejected without changing the existing echo file.

The VS Code environment at commit `9dc0c892128ecb0a7d8b2770f777038f70f34802` was verified on Windows PowerShell 5.1 on the same date, using VS Code 1.147.0, C/C++ 1.35.6, CMake Tools 1.25.10 and the existing GCC/CMake/Ninja tools plus GDB 15.2. Checks covered repeated Setup, preservation of unowned configuration, clean configure/build, 3/3 CTest, repeated fresh Run exports, a 400-angle Scan and preservation of previous debug output. GDB command-line debugging stopped at `main.cpp` and completed normally. The trusted workspace and C++/CMake views were observed in VS Code, but GUI task shortcuts and F5 were not tested end-to-end. The C++ simulator and CMake files were unchanged by this environment work, so the earlier numerical evidence retains its original scope.

Viewer implementation commit `4acdeeffcec3a9f4ffff08f72faad00c177fa874` passed 4/4 CTest with the viewer enabled and 3/3 in a fresh viewer-disabled build on the same Windows toolchain. Viewer model checks cover marker interpolation, transformed apparent position, caching, invalid drafts, angle wrapping and empty returns. Build warnings, formatter checks and PowerShell parsing passed; independent review confirmed the pinned raylib revision and absence of raylib fetching in the disabled build. An eight-frame OpenGL smoke run rendered successfully and its screenshot was visually inspected; the interactive window was also launched and observed. Automated mouse/keyboard interaction tests were not performed. The numerical core is unchanged by the viewer.

Formatting was checked on all six handwritten C++ files with CLion's bundled clang-format 20.0.0git (`fd2c1bf18bb22baa1ad38834dce4947d891f67d1`). From this folder:

```powershell
$formatTool = "$clionRoot\bin\clang\win\x64\bin\clang-format.exe"
Get-ChildItem include,src,tests -Recurse -File -Include *.hpp,*.cpp | ForEach-Object { & $formatTool --dry-run --Werror $_.FullName }
```

Scattering/image implementation `cac4c46eaeb3673eb2e3ae03cdbe82ae06a90545` passed 5/5 viewer-enabled and 3/3 viewer-disabled CTest, format and build checks on the same Windows toolchain. Tests cover roughness energy bounds/angles, particle zero/off/seed/beam/timing/pose, fixed scan reference/cache and empty-channel transitions, plus geometric timing/reflection/history links. Independent CSV inspection checked 51,799 echoes including 955 volume returns and 240,000 bins for finite three-channel sums, IDs, range and timing. Eight-frame scattering and wavefront screenshots were visually inspected; complete GUI control automation was not performed. Earlier commits above are historical evidence; the core and example configuration have now changed.


Native layout/camera/scan and analytic-front revision `a658584c17e1a642dc8e0b24b95cdac80ec34cc0` passed 5/5 CTest and independent review/format checks on the same Windows toolchain. Renders at 1000×700, 1280×800, 1600×900 and 1920×1080, including help/settings, were inspected. The latest front supersedes the earlier retained-contact truncation. Core scattering/config/CSV are unchanged from `cac4c46`; full GUI control automation was not performed.

These are numerical/software checks, not pool or hardware validation. Asymmetric multipath, enumeration of purely specular closed paths, interference, diffraction, side lobes, resolved rough geometry and electronics are absent. Frequency affects only the roughness heuristic; no wave field or hardware protocol is solved. Real-data classification, scan motion and calibration remain unverified.

Step 2 provides the educational viewer and a small draft configuration panel. Synthetic IMU/DVL data, sensor fusion and further simulation work remain later steps; advancement is controlled by the user.

## Sources

- [Blue Robotics Ping360 specifications](https://bluerobotics.com/store/sonars/imaging-sonars/ping360-sonar-r1-rp/): nominal beam widths, minimum/maximum range and motor angular resolution.
- [Ping360 protocol](https://docs.bluerobotics.com/ping-protocol/pingmessage-ping360/): angle and sampled return-strength interface; this simulator exports its own data model.
- [Cerqueira et al., ray-tracing sonar simulation](https://arxiv.org/abs/2001.03539): research background, not a claim that this implementation reproduces that simulator.
- [AUV wiki](https://github.com/Sub-Horizon-NTNU/AUV/wiki/Sonar-simulering): project status and numerical verification.
