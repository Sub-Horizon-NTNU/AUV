# Sonar simulation context

- Read [README.md](README.md) for commands, equations, units and limits before changes.
- Implemented scope: step 1 numerical Ping360 approximation and step 2 educational
  raylib viewer/configuration drafts. AUV dynamics, synthetic IMU/DVL and fusion
  remain later work requiring a new request.
- This folder's Norwegian name and root location are explicit user choices.
- Source/comments/repository documentation use English. The established AUV wiki
  remains Norwegian under its project documentation convention.

## When to read engineering skills

For routine development (including setup, debugging, and small fixes), use the
relevant project files and run the checks affected by the change. Start work
directly. A possible later push, review, or documentation task does not trigger
its skill during implementation.

Read only the skill for the stage in scope, when that stage starts:

| Stage | Skill |
| --- | --- |
| Explicit request to coordinate multiple roles | `subhorizon-engineering` |
| C++ cleanup | `subhorizon-cpp` |
| Documentation | `subhorizon-documentation` |
| Code review, including review for a significant change being prepared for push or PR | `subhorizon-code-review` |

For a routine push or PR, check the branch, diff, and relevant test results before
publishing within the user's authorization. Read specialist skills only for
stages that apply to the actual change.

Resolve each name to `skills/<name>/SKILL.md` in
[engineering-standards v0.2.0](https://github.com/Sub-Horizon-NTNU/engineering-standards/tree/7b110ab27ae68c377d52870f3882eca33b5307e9),
or use its installed copy at that revision. See the local checkout under
`~/.local/share/sonar-dev/engineering-standards` if present.

## How much verification to run

Pick the smallest check that can catch a plausible regression from the change,
and run any checks required by the project or the user. For prose or instruction
edits, inspect the diff and check any changed links or commands. For a narrow code
change, start with the relevant existing test or smoke check. Broaden coverage
when changes affect shared behavior or build tooling, when a failure needs
diagnosis, or when the focused checks leave a material risk untested.

Reuse passing results while the relevant code, inputs, and environment remain
the same. Once the selected checks pass, stop. Rerun only affected checks after
a relevant change or to investigate a failure. When preparing a push or PR,
reuse results that still apply to the final diff. Add a test only when required
or when existing coverage misses a concrete regression risk.

## Code owners and contracts

- `include/sonar/simulator.hpp`: value-based numerical API and explicit model limits.
- `src/simulator.cpp`: validation, fan quadrature, box intersections, reciprocal
  backscatter, specular continuation and profile integration. Keep independent of I/O.
- `src/io.cpp` and `include/sonar/io.hpp`: key/value configuration and CSV exports.
- `src/main.cpp`: CLI; `examples/pool.cfg`: illustrative defaults, not calibration.
- `tests/test_simulator.cpp`: analytic paths, energy, bounds, orientation and errors.
- `visualization/model.*`: cached ping, selection, draft validation, playback/path
  calculations; `view.*`: raylib rendering/controls; `main.cpp`: CLI and graphics RAII.
  `visualization/test_model.cpp` checks model behavior without a graphics window.
- Root `Sonar.code-workspace`, `.vscode/tasks.json` and `scripts/dev.ps1`: Windows
  VS Code setup, build/test/run tasks and local tool discovery. Setup owns generated,
  ignored editor settings/launch files; preserve unowned files. Do not commit local paths.
- Pool frame is right-handed, z up. Local launch angles differ from pool-frame
  path coordinates. Preserve SI units and explicitly suffixed degree/gradian inputs.
- API default is reciprocal; example uses geometrically gated receiver paths. Preserve complete
  TX-to-RX polylines, including aperture endpoints, and the distinction between apparent range and roundtrip length.
- Echo labels are simulation truth, not measured-data classification. Energy is relative.
- CSV writes preserve existing output files but are not atomic across the three files.

## Checks and remaining work

- VS Code helper actions `Setup`, `Build`, `Test`, `Run`, `Scan`, `Debug` and `View`, plus
  manual CMake/formatter commands, are in README. The helper uses `build-vscode`,
  creates fresh Run/Scan outputs, and archives prior debug output without deletion.
  Run checks affected by edits; keep generated builds and output untracked.
- Recorded baseline: implementation commit
  `9c5c44c95c9501bacfbb26b6d166e85afc3627a7`, Windows 2026-11-24,
  GCC 13.1/CMake 3.30.5 Debug: warning-free build, 3/3 CTest and six-file formatter
  check passed. Independent full-scan CSV/path checks passed; see README for scope.
- Environment baseline `9dc0c892128ecb0a7d8b2770f777038f70f34802`, same date:
  Windows PowerShell 5.1 setup/ownership/build/test/run/scan/debug-output checks
  passed, including 3/3 CTest and GDB command-line breakpoint/normal exit. VS Code
  workspace views were observed; GUI task shortcuts/F5 were not tested end-to-end.
  Simulator C++ and CMake remain unchanged from the numerical baseline.
- Viewer baseline `4acdeeffcec3a9f4ffff08f72faad00c177fa874`, same date/toolchain:
  viewer ON 4/4 CTest, fresh OFF 3/3, formatting/parsing and eight-frame OpenGL render
  passed. Screenshot/window inspected; no automated mouse/keyboard interaction tests.
  `SONAR_BUILD_VIEWER` defaults OFF; only ON fetches pinned raylib 6.2 and enables C.
  View enables it; other helper build actions disable it. Numerical core is unchanged.
- Current increment `cac4c46eaeb3673eb2e3ae03cdbe82ae06a90545` changes core scattering,
  example config, CSV and viewer: 5/5 ON, 3/3 OFF and independent CSV/render checks
  passed. Earlier baseline statements apply only to their commits. See README limits.
- `visualization/wavefront.*` owns the geometric one-way front. It propagates through
  ideal mirror folds for the full Rmax/c window, independent of echo energy/hit budget.
- Model caches 400 profiles/cloud and fixed combined scan reference. Channel changes
  must reselect eligible echoes, including transitions from an empty particle channel.
- Those results do not validate later source changes, another platform or pool acoustics.
- Known gaps: real dimensions/material calibration, asymmetric and purely specular
  return-path models, frequency/phase effects and measured Ping360 comparison.
- Keep viewer cache/state separate from rendering and graphics dependencies out of
  `sonar_core`. Marker motion is slowed playback, the purple point is an apparent
  location, and image/profile dB share the strongest combined bin over the entire scan.

- Current UI/front baseline `a658584c17e1a642dc8e0b24b95cdac80ec34cc0`: 5/5 CTest,
  review/format and multi-size render inspection passed; no complete GUI input automation.
  `layout.*` owns native sizing; model owns camera, scan clock and marker visibility.
  Static front geometry uses analytic folds/seam links; animated snapshots use 8×64. Particle visibility is display-only.
- Alignment/direction baseline `238c3c27482d5283f0014f45f5387a9c57f80a01`: 5/5 tests,
  independent review/format and screenshot checks. Alignment is full-pose sonar-local
  Top only; 3D remains pool-frame. Front arrows use folded propagation derivatives.
- Current baseline `6b26359`: received specular/diffuse paths, all-return clock and
  stationary XY movement; 5/5 CTest, 12 numerical groups, ten-file format and independent
  export checks passed. No live mouse testing. See README for scoped limits.
- In receiver_paths, the historical max_outward_hits key bounds total wall contacts;
  use actual path length/time and stored outbound/return split, not route reversal.
  Effective receiver radius is uncalibrated. Preserve API reciprocal compatibility.
- Move release rebuilds one stationary scan and synchronizes draft pose. Wavefront
  remains an independent ideal one-way diagnostic, not the received-echo transport.

- Playback/inference revision 0404a97 checked 2026-11-25: optimized build-vscode 6/6 CTest.
  Immutable emissions retain launch geometry/age (max 32); Loop/display duration,
  physical view window and TX pulse width have distinct meanings. Fade is a proxy.
- `visualization/inference.*` consumes only combined profiles/acquisition widths;
  rectangular level-scan prior, no pool/pose/Echo truth. Keep raw and estimated gain
  identical. Partial/fallback results and coherent-ghost ambiguity are expected limits.

- Current estimate/coords/route revision 150c7a2: optimized build-playback-opt 7/7
  CTest and 1000×700 screenshot check. Filter strength changes cached inference/masks,
  not gain or physics. Coordinates are paired-wall centre displacement, not a global map.
  Route is 48 stationary ellipse snapshots at 0.5 s intervals; manual Move stops it.
