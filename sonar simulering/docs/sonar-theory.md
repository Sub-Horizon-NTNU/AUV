# From a sonar ping to an image

This guide follows one active sonar ping from transmission to the range profile and then to the 360° image. It uses the rectangular pool in the Ping360 simulator as a worked example. See the [system guide](simulator-system.md) for the code and the [README](../README.md#calculation) for the implemented model equations.

## Time of flight and apparent range

An active sonar emits a short acoustic pulse and listens for returning energy. The Ping360 rotates a single transducer mechanically. Each ping produces a series of return-strength samples at one head angle. The nominal beam spans 2° horizontally and 25° vertically, so a heading is a narrow horizontal sector with much less information about elevation. In the simulator the fan is sampled with weighted rays. The example uses 3 horizontal × 11 vertical rays per heading. [Ping360 specifications](https://bluerobotics.com/store/sonars/imaging-sonars/ping360-sonar-r1-rp/) · [Ping360 protocol](https://docs.bluerobotics.com/ping-protocol/pingmessage-ping360/)

With constant sound speed $c$, a path of total length $L$ returns after $t=L/c$. The axis on a monostatic sonar display is **apparent range**, $r=ct/2=L/2$. The factor of two accounts for the trip out and back. At $c=1480$ m/s, a wall 5 m away on a straight round trip gives $L=10$ m and $t=6.76$ ms. If sound speed is estimated incorrectly, the inferred range scales with the chosen speed for the same measured delay.

## Reflections and multipath

A smooth surface produces specular reflection. The angle to the normal is preserved and the normal component of the ray reverses. A rough boundary can scatter energy in other directions, including toward the receiver. Particles or bubbles can also scatter within the water. The model gives these mechanisms distinct treatments and uses relative energy coefficients. At 750 kHz and 1480 m/s the wavelength is about 1.97 mm. Surface roughness enters the model through its ratio to wavelength and the angle of incidence.

![Pool plan with a straight round trip and an extra wall reflection, beside a pulse whose overlap contributes 56% and 44% to adjacent range bins](figures/paths-and-bins.png)

**Left.** The straight path is 14 m in total and appears at 7 m. The dashed return via the upper wall gives a total path of about 17.63 m and appears at 8.82 m. This sketch isolates geometry. The dashed path arrives about 49° off the outgoing axis, which puts it outside the example receiver beam for that heading. Real echoes that do pass the receive gate can overlap at the same apparent range. Bright points and extended wall-like patterns can therefore arise from more than one reflection.

Received strength depends on spreading and absorption along both legs, on how much energy is scattered toward the sensor, and on the sensor response. For an ideal point scatterer with spherical spreading and equal leg lengths, the two intensity factors scale as $1/r^4$ before absorption. Extended pool walls and reverberation can behave differently. Simulator values are relative energies from the selected route and scattering models. The hardware supplies sampled return-strength bytes shaped by its receiver and gain. [Ainslie and Leighton, active sonar equation and two-way propagation loss, pp. 27–28](https://eprints.soton.ac.uk/398067/) · [Ping360 protocol](https://docs.bluerobotics.com/ping-protocol/pingmessage-ping360/)

## Pulse duration, range bins and the image

The simulator spreads each in-window arrival over a causal rectangular pulse. At pulse duration $\tau$, its width on the range axis is $c\tau/2$. The example's 20 µs pulse spans **14.8 mm**, while its 600 bins over 20 m are **33.3 mm** wide. The two scales describe different things. Pulse width sets how far one arrival spreads in this model, and bin width sets how finely that spread is sampled. Actual range resolution also depends on the transmitted waveform, bandwidth and receiver processing.

**Right panel above.** In a schematic grid with 33.3 mm bins, the 14.8 mm pulse starts just before a bin boundary. About 56% of its energy falls in the first bin and 44% in the next. The simulator assigns energy by this overlap and clips pulse tails at the maximum range. An arrival outside the receive window contributes zero to the profile.

![Polar plot of combined simulated returns across 400 headings, with wall-like traces and scattered points](figures/example-scan.png)

Each radial slice is a profile from one stationary heading, spaced by 0.9°. The plot uses `total_intensity` from a scan of [the example pool](../examples/pool.cfg), on an 80 dB scale referenced to the strongest combined bin in that scan. Rings mark 5 m increments of apparent range, with 20 m at the outer edge. The inner rectangle follows the main wall returns, while additional traces and dots show how multipath and simulated particles complicate the image. Keep the reference level in mind when comparing brightness across scans.
