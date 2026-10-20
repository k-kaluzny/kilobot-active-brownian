# Kilobot – Active Brownian Motion

Controller, simulation setup and (in progress) trajectory-tracking tools for a swarm of
[Kilobots](https://www.kilobotics.com/) performing a discrete approximation of active Brownian motion.
The same C code runs in the [Kilombo](https://github.com/JIC-CSB/kilombo) simulator and on physical robots.

This repository accompanies a paper in preparation on finite-size effects in interacting robot swarms,
comparing particle trajectories (simulated and from Kilobot experiments) with a mean-field PDE description.

> **Status:** work in progress. The controller and the single-robot simulation are working, and the
> controller has been used in real-robot experiments. Trajectory tracking from video (local and
> GPU cluster) is being added.

## The model

Every robot runs the same discrete run-and-turn protocol, independently of the others. The robot program
runs at 32 ticks per second.

| State | Behaviour |
|-------|-----------|
| **Run** | Move forward using the calibrated motor command. The run ends at the first of: (a) a successful tumble check (one check every Δt_c, succeeding with probability p_c), (b) the hard timeout τ_r, or (c) a neighbour detected within R_s, which triggers avoidance instead of a turn. |
| **Turn** | After a tumble: rotate for τ_t in a random direction (clockwise or counter-clockwise, equal probability), then resume Run. |
| **Avoid** | After a neighbour detection: rotate for τ_a in a random direction, then resume Run. |

| Parameter | Value |
|-----------|-------|
| Tumble check interval Δt_c | 0.344 s |
| Tumble probability per check p_c | 0.6 |
| Hard run timeout τ_r | uniform in [0.625, 1.844] s (mean 1.23 s) |
| Effective run duration | ≈ 0.54 s (92% of runs end at a tumble check, 8% at the timeout) |
| Turn duration τ_t | uniform in [1.5625, 2.969] s (mean 2.27 s) |
| Avoidance rotation τ_a | uniform in [0.625, 1.531] s (mean 1.08 s) |
| Sensing radius R_s | 50 mm centre-to-centre (17 mm edge-to-edge) |

In simulation each state has its own RGB LED colour, so state transitions can be checked visually. The same code
is then flashed to the physical robots, where motor variability, battery level, surface friction and the
noise of vibration-based locomotion require extra parameter tuning.

### Validation: ballistic-to-diffusive crossover

With a single robot there are no robot–robot or boundary effects in the statistics. The
time-averaged mean-square displacement (TAMSD) of a trajectory is

$$\overline{\delta^2}(s) = \frac{1}{N_t - m}\sum_{k=0}^{N_t-m-1} \left|\mathbf{x}(t_{k+m}) - \mathbf{x}(t_k)\right|^2, \qquad s = m\,\delta t,$$

and the local exponent $\alpha(s) = d\log\overline{\delta^2}/d\log s$ should decay from about 2 (persistent,
ballistic motion at short lags) to about 1 (diffusive motion once the heading has decorrelated). This is the
signature of active Brownian motion. `controllers/kilombo.json` is set up for this single-robot run.

<!-- TODO: add the alpha(s) plot as docs/alpha_validation.png and reference it here. -->

## What is here

```
controllers/
├── active-brownian.c       # robot controller
├── active-brownian.h
├── Makefile.osx            # builds the simulator binary and the real-robot .hex
├── kilombo.json            # Kilombo simulation settings (single-robot validation run)
└── start_positions.json    # initial robot positions
```

## Requirements

**Simulation (Kilombo)**
- macOS (the Makefile links the Cocoa framework; a Linux Makefile is not yet provided)
- [Kilombo](https://github.com/JIC-CSB/kilombo), installed so that `kilombo.h` is in
  `/usr/local/include` and `libsim` is in `/usr/local/lib`
- SDL 1.2 and [jansson](https://digip.org/jansson/), which Kilombo depends on
- A C compiler (`gcc`)

**Real robots**
- Kilobots (K-Team) and the overhead controller
- AVR toolchain (`avr-gcc`, `avr-binutils`, `avr-libc`)
- [kilolib](https://github.com/acornejo/kilolib), built so that `build/kilolib.a` exists
- [KiloGUI](https://github.com/acornejo/kilogui) to upload programs, calibrate and control the robots

## Build and run

Simulation:

```bash
cd controllers
make -f Makefile.osx sim
./active-brownian          # Kilombo reads kilombo.json from the current directory
```

Real Kilobots (produces `active-brownian.hex`):

```bash
make -f Makefile.osx hex
```

If Kilombo or kilolib are installed elsewhere, override the paths:

```bash
make -f Makefile.osx KILOHEADERS=/path/to/kilolib SIMHEADERS=/path/to/kilombo/include sim
```

Build output (`*.o`, `*.elf`, `*.hex`, the binary) is not tracked; `make -f Makefile.osx clean` removes it.

## Real-robot experiments

- **Calibration.** Kilobots differ in speed and turning because of battery level, motor variability and
  small mechanical asymmetries, so each robot is calibrated in KiloGUI before every batch. Four motor
  parameters are set per robot: clockwise turn, anticlockwise turn, and the left and right motor forces
  for forward motion. Robots are fully charged and run freely for about 30 s to warm up the motors.
- **Arena.** Flat 1180 × 950 mm whiteboard, recorded from above with a fixed Logitech C920 webcam
  at about 25 frames per second (recorded with Guvcview, trimmed in Shotcut).
- **Swarm sizes.** N ∈ {10, 15, 20, 25}. Each recording is trimmed so the robots start moving at the
  2 s mark and lasts 302 s (300 s of motion).
- **Initial condition.** Robots start on a circle centred at (590, 475) mm, with 10 mm
  edge-to-edge spacing between neighbours.
- **Tracking.** The robot LEDs are switched off in the recorded experiments, so detection relies on the
  robot bodies seen from above. Positions are extracted from the video and converted to millimetres using
  the arena dimensions. The tracking code will live in this repository (see roadmap).

## Known limitations

- Only a macOS Makefile exists; Linux linking flags differ (no Cocoa framework).
- Paths assume Kilombo under `/usr/local`. Homebrew on Apple Silicon uses `/opt/homebrew`, so set
  `SIMHEADERS` and the library path accordingly.

## Roadmap

- [ ] Single-robot trajectory tracking from video (local)
- [ ] Batch tracking on a GPU cluster (Slurm)
- [ ] Shared trajectory format for simulated and experimental data
- [ ] Analysis scripts (TAMSD, α(s), empirical density)

## License

MIT, see [LICENSE](LICENSE).
