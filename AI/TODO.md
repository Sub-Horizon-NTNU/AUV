# Sonar to-do list

Record work here when it is deferred to another PC or session. Add the next concrete action and any dependency. This list tracks pending work and does not start it.

## Deferred work

- [ ] Use mirrored-receiver reference paths to measure higher-order capture error and study fan overlap for direct returns. See [method and validation notes](SONAR-FOLLOWUPS.md).
- [ ] Profile the 400-heading scan before choosing performance work. See [performance notes](SONAR-FOLLOWUPS.md#performance).
- [ ] Plan the first non-box validation scene with one submerged obstacle and a known acoustic shadow. See [advanced validation notes](SONAR-FOLLOWUPS.md#validation-beyond-the-pool).

<!-- Entry format: - [ ] Task. Next: concrete action. Needs: dependency, if any. -->

## Candidates from existing project notes

These are known gaps or later steps from the [project instructions](../sonar%20simulering/AGENTS.md) and [simulator README](../sonar%20simulering/README.md). Move an item to **Deferred work** when the user chooses it.

- [ ] Check the VS Code workflow end to end on a Windows development PC. Exercise the build shortcut, visualizer task and F5 with an actual breakpoint.
- [ ] Measure the pool dimensions and the sonar pose. Save the values and acquisition settings with the measurements before comparing profiles.
- [ ] Compare measured Ping360 returns to the example simulation. Start with a level stationary ping at a known wall distance, then use full scans to assess the scattering and wall estimates.
- [ ] Consider synthetic IMU and DVL data and sensor fusion when the project advances beyond stationary sonar simulation.
