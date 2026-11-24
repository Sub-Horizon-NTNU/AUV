# Sub Horizon — Sonar simulation

The `sonar` branch contains the standalone C++17 Ping360 numerical simulator.

- [Setup, configuration and model documentation](sonar%20simulering/README.md)
- [CMake project](sonar%20simulering/CMakeLists.txt) — open `sonar simulering` in CLion.
- [Project wiki](https://github.com/Sub-Horizon-NTNU/AUV/wiki/Sonar-simulering)

Step 1 provides numerical echo paths and range profiles for a configurable rectangular pool.
The model uses a limited reciprocal-backscatter approximation and has not been calibrated
against pool measurements. Raylib visualization and synthetic IMU/DVL data are later steps.
