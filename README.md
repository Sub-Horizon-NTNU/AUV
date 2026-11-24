# Sub Horizon — Sonar simulation

The `sonar` branch contains the standalone C++17 Ping360 numerical simulator.

- [VS Code workspace](Sonar.code-workspace) — open this file for the prepared build, run and debug tasks.
- [Raylib viewer quickstart](sonar%20simulering/README.md#interactive-raylib-viewer-step-2) — choose `Sonar: Visualize` in VS Code.
- [Setup, configuration and model documentation](sonar%20simulering/README.md)
- [CMake project](sonar%20simulering/CMakeLists.txt) — open `sonar simulering` in CLion.
- [Project wiki](https://github.com/Sub-Horizon-NTNU/AUV/wiki/Sonar-simulering)

Step 1 provides numerical echo paths and range profiles for a configurable rectangular pool.
The model uses a limited reciprocal-backscatter approximation and has not been calibrated
against pool measurements. Step 2 compares ray paths, a simulated 400-angle sonar image
and a coarse geometric wavefront, with optional surface/particle scattering.
Synthetic IMU/DVL data and sensor fusion remain later steps.
