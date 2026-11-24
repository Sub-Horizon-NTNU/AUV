# Sonar simulation context

- Read [README.md](README.md) for commands, equations, units and limits before changes.
- Scope: step 1, stationary numerical Ping360 approximation. Do not advance to
  raylib visualization, AUV dynamics, synthetic IMU/DVL or fusion until requested.
- This folder's Norwegian name and root location are explicit user choices.
- Source/comments/repository documentation use English. The established AUV wiki
  remains Norwegian under its project documentation convention.
- Standards: engineering-standards v0.2.0, commit
  `7b110ab27ae68c377d52870f3882eca33b5307e9`. Use
  [subhorizon-engineering](https://github.com/Sub-Horizon-NTNU/engineering-standards/blob/7b110ab27ae68c377d52870f3882eca33b5307e9/skills/subhorizon-engineering/SKILL.md).
  All nine sibling skills are installed on the current machine under
  `C:/Users/luanl/.codex/skills`; elsewhere obtain the same complete revision.

## Code owners and contracts

- `include/sonar/simulator.hpp`: value-based numerical API and explicit model limits.
- `src/simulator.cpp`: validation, fan quadrature, box intersections, reciprocal
  backscatter, specular continuation and profile integration. Keep independent of I/O.
- `src/io.cpp` and `include/sonar/io.hpp`: key/value configuration and CSV exports.
- `src/main.cpp`: CLI; `examples/pool.cfg`: illustrative defaults, not calibration.
- `tests/test_simulator.cpp`: analytic paths, energy, bounds, orientation and errors.
- Root `Sonar.code-workspace`, `.vscode/tasks.json` and `scripts/dev.ps1`: Windows
  VS Code setup, build/test/run tasks and local tool discovery. Setup owns generated,
  ignored editor settings/launch files; preserve unowned files. Do not commit local paths.
- Pool frame is right-handed, z up. Local launch angles differ from pool-frame
  path coordinates. Preserve SI units and explicitly suffixed degree/gradian inputs.
- Only reciprocal terminal-backscatter paths are represented. Preserve complete
  closed polylines and the distinction between apparent range and roundtrip length.
- Echo labels are simulation truth, not measured-data classification. Energy is relative.
- CSV writes preserve existing output files but are not atomic across the three files.

## Checks and remaining work

- VS Code helper actions `Setup`, `Build`, `Test`, `Run`, `Scan` and `Debug`, plus
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
- Those results do not validate later source changes, another platform or pool acoustics.
- Known gaps: real dimensions/material calibration, asymmetric and purely specular
  return-path models, frequency/phase effects and measured Ping360 comparison.
- The direct library API and complete paths are the intended basis for the next
  raylib step. Do not add graphics dependencies to `sonar_core`.
