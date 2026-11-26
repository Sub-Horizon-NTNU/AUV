# Sonar method, performance and validation

The present simulator is an inspectable geometric model for a stationary Ping360-style sonar in a box. Use [the system guide](../sonar%20simulering/docs/simulator-system.md) for the current algorithm and the [to-do list](TODO.md) for work to resume.

## What the comparison found

- At `63a9921` the normal wall return in `examples/pool.cfg` changed from one specular echo at 3×11 rays to zero at 3×12. The diffuse energy stayed near 4.10 × 10⁻⁸. The odd grid contains an axial ray and the even grid misses the small capture sphere. The single-wall specular path now uses the exact wall normal and a solid-angle energy approximation independent of the grid.
- The old 5 cm capture sphere shortened a normal 10 m round trip to 9.95 m. Direct specular returns now use centre-to-centre timing. Sampled higher-order returns use closest approach inside the aperture. They still depend on the grid and keep only the first eligible higher-order capture per ray.
- The single-wall energy estimate uses the aperture cone's solid angle divided by the transmit fan's solid angle, capped at one. It tests the cone axis against the fan. At a beam edge the actual angular overlap changes continuously, so the current response can jump. Treat the energy as a relative model estimate. Future work can integrate aperture overlap with the transmit and receive patterns.
- The rectangular box makes mirrored-receiver paths a useful independent reference. Enumerate image points for a few reflection orders, reconstruct plane contacts, check both beam angles and compare path lengths and detections across odd and even ray grids. Use energies for calibration only after the geometry check. [Allen and Berkley, image method](https://doi.org/10.1121/1.382599)

## Performance

1. Measure runtime first. Use a release-style build and the same pool, pose, ray counts, particle count and range bins. Time one ping, the 400-heading computation, CSV writing, inference and texture updates separately. Track elapsed time and allocations alongside the resulting profiles.
2. Reuse pose-local data within a scan. The fixed particle cloud, local particle directions and ranges, and the rotation basis can be computed once per pose/configuration. The current per-ping API can stay usable on its own, with a separate prepared context only if profiling warrants it.
3. If computing 400 independent headings dominates, parallelize by heading with deterministic output order. Keep CSV I/O and rendering outside the workers. Compare numerical results with the serial path before relying on timings.
4. If triangle meshes become the bottleneck, benchmark a bounding-volume hierarchy for first intersections and return visibility. Cerqueira et al. use GPU rasterization for first hits and selectively trace later reflections. HoloOcean groups nearby visible surfaces before estimating multipath. Consider these approaches after measuring mesh traversal and choosing which echoes must be preserved. [Cerqueira et al.](https://arxiv.org/html/2001.03539) · [Potokar et al.](https://robots.et.byu.edu/jmangelson/pubs/2022/Potokar22iros.pdf)

## Validation beyond the pool

1. **Anchor the box model.** Collect repeated stationary pings from a flat wall at a known range and several incidence angles. Record sensor pose, sound speed or water temperature, transmit duration, sample interval and gain. Compare first-arrival range, pulse width and relative levels before adding clutter. Keep some runs separate for checking the chosen parameters.
2. **Test a known obstruction.** Add a simple submerged target in front of a wall. Verify first-hit ordering and transmit and return visibility separately. Compare the expected shadow and changes in the wall echo, then move the target so one return leg is blocked while the outgoing leg is clear.
3. **Add complexity one at a time.** Use a tilted plane for angle and coordinate checks, then a box or corner reflector to isolate multi-bounce arrivals. For mesh scenes, test hit tolerances, normals, occlusion and path reconstruction against small scenes with analytic answers before importing a large environment.
4. **Test the sensor and estimator separately.** For advanced imagery, assess beam shape, material response, gain, noise and speckle using repeated Ping360 data. For moving scans, give each ping a timestamp and pose, then compare wall estimates against surveyed geometry using held-out trajectories. Report missed walls and spurious walls as well as position error. A rectangular prior can fit a ghost or fail in a non-rectangular scene. [Ping360 protocol](https://docs.bluerobotics.com/ping-protocol/pingmessage-ping360/) · [HoloOcean noise and multipath](https://robots.et.byu.edu/jmangelson/pubs/2022/Potokar22iros.pdf)
