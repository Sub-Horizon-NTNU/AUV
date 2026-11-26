# Cross-PC handoff

Use this folder for context that should travel with the `sonar` branch. Put open work in [TODO.md](TODO.md). Keep this file short and update it when the project state, a decision or a useful verification result changes.

## Resume on another PC

1. Check out `sonar` and sync with `origin/sonar`. Check the working tree before modifying files.
2. Open [Sonar.code-workspace](../Sonar.code-workspace). For a fresh Windows checkout, follow the [VS Code setup instructions](../sonar%20simulering/README.md#vs-code-on-windows). The setup helper discovers locally installed GCC, GDB, CMake/CTest and Ninja and generates editor settings for that machine.
3. Use the [project instructions](../sonar%20simulering/AGENTS.md) for code conventions and how much verification to run. Read the specialist instructions only when the task calls for them.

Tool paths, generated VS Code settings, build caches and exported data are local to each machine. Keep those and credentials out of this folder. If another person or PC needs an artifact, record its purpose and how to reproduce or obtain it.

## Current project state

- `sonar_core` traces a Ping360-style fan in a rectangular pool. The CLI exports paths, echoes and range profiles. The optional raylib viewer shows those results and fits candidate wall sections from the simulated scan.
- The numerical API defaults to reciprocal returns. The example pool selects receiver-path tracing, roughness and particle scattering. The example parameters and energies are illustrative. Compare against measured data before drawing conclusions about real pool performance.
- [Sonar theory](../sonar%20simulering/docs/sonar-theory.md) explains the physics and plots. The [system guide](../sonar%20simulering/docs/simulator-system.md) maps the numerical pipeline to code and includes a reproducible single-wall check.
- At `0678ea1`, the documented wall example was run on Windows. It produced a 9.95 m return path, 4.975 m apparent range and one occupied bin at index 149. See the project instructions and simulator README for previous verification tied to their revisions.

## Record a handoff

When pausing work, leave enough information to resume: the branch or commit, the last useful result, the next concrete action and any blocker. Mention local-only files if another PC needs their contents. Keep incomplete tasks open in the to-do list and record a result or commit when closing them.
