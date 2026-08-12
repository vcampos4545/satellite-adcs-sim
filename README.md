# main

A closed-loop cubesat ADCS (attitude determination and control system) / flight-software simulation: a single high-fidelity satellite, its sensors, its actuators, and the flight software driving them, all running against a real 6-DOF rigid-body physics engine.

**Scope.** This project answers single-vehicle GNC/flight-software questions — attitude estimation and control, actuator allocation and fault handling, momentum management, and (now) real single-satellite orbital dynamics — for one satellite at a time. The scale boundary is the point, not whether orbital motion exists at all: it deliberately does not do orbit-_level mission design_ (constellation architecture, coverage across many satellites, station-keeping maneuver planning, conjunction screening); that's a different fidelity/scale problem (one high-fidelity vehicle vs. thousands of low-fidelity trajectories) solved by a separate project, [constellation-sim](https://github.com/vcampos4545/constellation-sim).

This project itself builds on [spacecraft-dynamics-sim](https://github.com/vcampos4545/spacecraft-dynamics-sim), a generic rigid-body physics engine (bodies, constraints, actuators, sensors, and — since this project needed real orbital dynamics — a double-precision `rigidbody/orbit/` module) with no spacecraft- or flight-software-specific code in it — fetched as a dependency, not vendored. See `docs/ALGORITHMS.md`'s "Orbital Mechanics" section for how this project uses it.

## What's simulated

**Sensors**: a 3-axis gyro+accelerometer IMU, a 3-axis magnetometer, and a star tracker — all modeled with realistic noise/bias, not oracle readings. The star tracker also models two real failure modes flight software has to handle as routine, not edge cases: going blind when the sun is too close to the boresight, and losing star lock while slewing too fast to centroid stars.

**Estimation**: a multiplicative EKF (state = attitude + gyro bias, propagated via strapdown quaternion kinematics) with the star tracker as the primary correction and a sun+magnetometer TRIAD solve as the fallback when it's unavailable — including live tracking of the estimator's own uncertainty, not just a point estimate.

**Actuators**: a 4-wheel reaction wheel pyramid and a 3-axis magnetorquer cluster, with:

- Three attitude controllers (PID, LQR, cascaded P/rate) selectable live, each auto-tuned from the vehicle's actual inertia and a settling-time/damping-ratio target.
- B-dot magnetic detumbling for post-deployment tumble recovery.
- Cross-product-law momentum desaturation, running automatically in the background (not a mode) whenever wheel saturation gets close, with headroom-aware gain scaling so the desaturation maneuver itself can't destabilize pointing.
- Manually-triggered wheel faults (degraded or dead, per wheel, from the Simulation tab) the allocator has to route around.

**EPS (electrical power subsystem)**: a body-mounted solar panel on each of the 6 faces (standard cubesat layout — whichever face happens to be sunward generates, no gimbal), each following the standard cosine law against the current sun direction and attitude, feeding a Coulomb-counting battery model. Every actuator/sensor draws a representative, physically-motivated amount of power each FSW cycle (wheels/magnetorquers scale with commanded effort, not a flat number) and is debited against the battery alongside whatever the panels generated that cycle — the battery's state of charge is a real consequence of what the spacecraft is doing, not a cosmetic number.

**FDIR / mode manager**: an autonomous fault-detection layer that runs every FSW cycle above guidance/control — the same "mode manager" vs. "GNC" split a real flight computer's task structure has. It watches wheel health telemetry, the EKF's own confidence, body rate, and battery state of charge, and can override the commanded pointing mode with a safe one (Sun-pointing — which also happens to maximize solar generation — or Detumble first if the rate itself is out of the controller's envelope) without waiting for ground intervention. Faults latch until explicitly acknowledged, autonomy itself is ground-inhibitable, and every trip/clear is timestamped in an onboard event log.

**Pointing modes**: Nadir, Sun-pointing, Detumble, Target, Slew (fast/coarse), Fine-pointing (slow/precise), and Reflect (aims the +Z mirror's normal to bounce sunlight onto `target` instead of pointing at it directly) — each with its own gain/rate-limit tuning rather than one-size-fits-all.

**GUI**: a single tabbed panel (FSW / Sensors / Actuators / FDIR / EPS / Simulation) covering live gain tuning, sensor telemetry with rolling plots, per-actuator status and manual override, FDIR status/thresholds/event log, battery/solar-generation telemetry, and simulation controls (pause, induced tumbles, fault injection, forced desaturation) — plus a 3D view with sun/target pointing-error lines, magnetic field visualization, and (for fun) a mirror on the +Z face showing sun-reflection geometry with the sun sized to its real ~32 arcminute angular diameter.

## Build

```bash
cmake -B build
cmake --build build -j$(nproc)
```

Fetches `rigidbody` (spacecraft-dynamics-sim), VGL, and Dear ImGui from GitHub — no local sibling checkouts required.

## Run

```bash
./build/main
```

**Controls**:

| Key       | Action                                                                 |
| --------- | ---------------------------------------------------------------------- |
| `1`-`7`   | Pointing mode: Nadir / Sun / Detumble / Target / Slew / Fine / Reflect |
| `Space`   | New random target (Target/Slew/Fine/Reflect modes)                     |
| `T`       | Kick the body into a random tumble (to test Detumble)                  |
| Left-drag | Orbit camera                                                           |
| Scroll    | Zoom                                                                   |

Everything else — controller algorithm/gains, detumble actuator, manual actuator override, desaturation thresholds, simulation pause — is in the ImGui panel.
