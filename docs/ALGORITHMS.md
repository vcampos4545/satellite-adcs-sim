# Algorithms, Math, and Model Assumptions

This is the standing reference for the equations, algorithms, and modeling
assumptions behind this project's flight software and physical models. Code
comments explain *why* a specific line does what it does; this document is
where the underlying math and its assumptions live in one browsable place,
independent of where in the code they're implemented.

**When to update this file**: whenever a change touches the math itself —
a new control law, a changed EKF term, a new fault model, a new physical
model (like EPS). A refactor that moves code around without changing the
equations doesn't need an update here. See `docs/TESTING.md` for the
matching rule on when a change needs a new/updated test.

Conventions used throughout: quaternions are `[w, x, y, z]` (scalar-first,
`glm::quat`'s own convention); all vectors are body-frame unless stated
otherwise; SI units unless stated otherwise (kg, m, s, rad, N·m, T, W, J).

---

## Coordinate Frames

This project has **no orbital mechanics** — no inertial-to-orbit frame
transform, no LVLH frame, no orbital rate. Everything is either:

- **World frame**: the simulation's fixed global frame. `target`,
  `sunPosition`, `ambientFieldWorld` are all given in this frame.
- **Body frame**: fixed to the spacecraft, origin at its center of mass.
  Sensor readings, actuator axes, and the EKF state are all in this frame.

`RigidBody::orientation` (a world-frame quaternion) is the only thing that
relates the two, via `world_vec = orientation * body_vec` (and
`body_vec = inverse(orientation) * world_vec`). Body +Z is the payload/
pointing axis every guidance mode aims (see "Guidance" below); body -Z is
the star tracker boresight, deliberately opposite it.

## Hardware-Abstraction Boundary

`ADCS`/`FDIR` (the flight software) never reference `RigidBody`,
`PhysicsWorld`, or any sensor/actuator simulation type — see
`src/FlightTypes.h`. `ADCS::step()` is a pure function of `(internal state,
FSWInputs, dt) -> FSWOutputs`. This is the seam a HIL rig or real flight
hardware would eventually replace one side of, and it's why `tests/`
can build FSW logic against nothing but `glm` (see `tests/CMakeLists.txt`).

---

## Guidance

Every pointing mode except `DETUMBLE` resolves to a `pointDir` (world
frame) and then solves for the quaternion that rotates body +Z onto it:

```
dot = bodyUp . pointDir                    (bodyUp = (0,0,1))
axis = normalize(bodyUp x pointDir)
angle = acos(dot)
targetAttitude = angleAxis(angle, axis)
```

with the degenerate `dot > 0.9999`/`dot < -0.9999` cases handled explicitly
(already aligned / exactly antipodal, where `cross()` is near-zero and the
axis is undefined).

`pointDir` per mode:

| Mode | `pointDir` |
|---|---|
| `NADIR` | fixed world `(0,0,-1)` — **not** a real Earth-center vector; this sim has no orbital position, so "down" is an honest fixed direction rather than a fabricated nadir |
| `SUN_POINTING` | `normalize(sunPosition - spacecraftPosition)` |
| `TARGET` / `SLEW` / `FINE_POINTING` | `normalize(target - spacecraftPosition)` |
| `REFLECT` | see below |
| `DETUMBLE` | none — bypasses guidance entirely, see "Detumble" |

### REFLECT: mirror bisector law

`REFLECT` aims body +Z (the mirror's normal) so a flat mirror there bounces
sunlight onto `target`, instead of pointing +Z at anything directly. For a
flat mirror, the normal that reflects a ray arriving from direction `u`
(mirror to source) into a ray leaving toward direction `v` (mirror to
target) is the bisector:

```
pointDir = normalize(normalize(sunPosition - spacecraftPosition)
                    + normalize(target - spacecraftPosition))
```

Proof sketch (see the `computeGuidance()` REFLECT case for the derivation
this mirrors): with `n = normalize(u+v)`, `reflect(-u, n) = -u + 2(u·n)n`,
and substituting `u·n = |u+v|/2` gives exactly `v`. Degenerate when `u` and
`v` are antipodal (`u+v = 0`, sun and target exactly opposite the
spacecraft) — a real flat mirror can't solve that geometry either, so
`normalize()`'s NaN there is a correct signal, not a bug to special-case.

Verified headlessly in `tests/` isn't currently present for REFLECT
specifically (it was validated numerically when built — reflected-ray-to-
target angle error of 0.000°, see the feature's commit) but doesn't yet
have a persisted regression test; a good first addition per
`docs/TESTING.md`.

---

## Attitude Estimation: Multiplicative EKF

State: `[attitude error δθ (3), gyro bias error δb (3)]`, an indirect
("error-state") multiplicative EKF — the nominal state is `estimatedAttitude`
(a full quaternion) and `gyroBiasEstimate`, propagated directly, while the
6x6 covariance describes uncertainty in a small-angle *error* around them.
This is the standard approach for attitude EKFs since quaternions have no
flat vector space to naively apply a linear Kalman filter to.

Covariance is stored as four `glm::mat3` blocks (`covAA`, `covAB`, `covBB`)
instead of one 6x6 matrix, since GLM has no fixed 6x6 type — `covBA` is
never stored separately (`P` is symmetric by construction, every update
preserves that).

### Propagate (`propagateEstimator`, every `step()` cycle)

Strapdown quaternion kinematics, bias-corrected:

```
omega = gyroMeasured - gyroBiasEstimate
estimatedAttitude += 0.5 * estimatedAttitude * quat(0, omega) * dt
estimatedAttitude = normalize(estimatedAttitude)
```

Covariance: first-order (Euler) discretization of the linearized
error-state dynamics

```
d(deltaTheta)/dt = -omega x deltaTheta - deltaBias
d(deltaBias)/dt  = 0                        (pure random walk)
```

i.e. `Phi = I + F*dt` with `F = [[-[omega x], -I], [0, 0]]` in block form,
hand-expanded as `Phi*P*Phi^T + Q*dt`:

```
phiAA = I - skew(omega)*dt
newAA = phiAA*covAA*phiAA^T - dt*covAB^T*phiAA^T - dt*phiAA*covAB + dt^2*covBB + gyroNoisePsd*dt*I
newAB = phiAA*covAB - dt*covBB
newBB = covBB + gyroBiasWalkPsd*dt*I
```

`gyroNoisePsd`/`gyroBiasWalkPsd` are continuous-time process-noise spectral
densities, set at `configure()` from fixed constants
(`GYRO_NOISE_STD_RAD_S = 0.0008`, `GYRO_BIAS_DRIFT_STD_RAD_S = 0.0003`)
chosen to numerically match the simulated `IMU`'s own default noise figures
— **an assumption**, not something FSW derives on its own; a real system
would get these from the gyro's datasheet.

### Correct (`correctEstimator`, shared by star tracker + TRIAD)

Given an absolute attitude measurement `qMeas` with isotropic covariance
`R` (rad²):

```
qErr = inverse(estimatedAttitude) * qMeas   (sign-flipped if qErr.w < 0)
dz = 2 * qErr.xyz                            (innovation, small-angle)
S = covAA + R*I                              (H = [I, 0], so H*P*H^T = covAA)
Ka = covAA * S^-1                            (top block of K = P*H^T*S^-1)
Kb = covAB^T * S^-1                          (bottom block)
dTheta = Ka * dz;  dBias = Kb * dz
```

Multiplicative reset: `estimatedAttitude = normalize(estimatedAttitude *
quat(1, 0.5*dTheta))`, `gyroBiasEstimate += dBias`. Covariance update is
the standard `P_new = (I - K*H)*P` expanded in blocks.

`attitudeUncertaintyDeg` (exposed for telemetry and consumed by FDIR) is
`degrees(sqrt(trace(covAA)/3))` — the RMS 1-sigma angle across all three
axes, not a true confidence ellipsoid, but adequate as a single scalar
"how much do I trust this" signal.

### TRIAD fallback (`computeTriadFallback` / `computeTriadAttitude`)

When the star tracker has no valid reading (sun-blinded or slewing too
fast), a deterministic two-vector solve using the magnetometer (primary)
and coarse sun sensor (secondary) forms a fallback absolute-attitude
measurement, corrected the same way as the star tracker (same
`correctEstimator()` call, different `R`).

- Guards against a near-singular solve: rejects the pair when
  `|cross(fieldDirRef, sunDirRef)| < 0.1` (references nearly parallel).
- `R` for the fallback is dominated by the coarse sun sensor's noise
  (`sunSensorNoiseRad²`) — an approximation that doesn't rigorously
  propagate both sensors' noise through the TRIAD solve, just uses the
  worse of the two as a stand-in.

---

## Attitude Control

Three interchangeable controllers, all taking `(targetAttitude, q_current,
omega_current, dt) -> torqueCommand`, auto-tuned from `settlingTime`/
`dampingRatio`/`omega_max` rather than hand-picked gains (see `Controllers.h`
`autoTune()` for each):

- **PID**: classic quaternion-error PID with integral clamping.
- **LQR**: per-axis double-integrator model (`theta_dot = omega, omega_dot
  = u/I_axis`; small-angle error, principal-axis inertia, cross-axis
  coupling ignored), closed-form continuous-time algebraic Riccati
  solution (no numerical CARE solver needed for a 2-state/1-input system).
- **Cascaded**: outer loop maps attitude error to a rate command (with
  saturation for large slews), inner loop maps rate error to torque.

Per-mode gain presets (`tuningForMode()` in `ADCS.cpp`): `SLEW` trades
precision for speed (short settling time, high rate cap), `FINE_POINTING`
trades speed for precision (long settling time, overdamped, tight rate
cap); everything else uses one moderate preset.

**Sign convention** (load-bearing, found by empirical testing, not just
derivation): `ReactionWheel`'s simulated reaction dynamics apply the
*negative* of whatever torque is commanded (Newton's-third-law reaction —
spinning the wheel one way reacts the bus the other way). Every control
law here computes the torque it wants applied *to the bus*, and
`allocateActuators()`/the wheel model's own convention handles the sign
flip — but hand-derived laws that skip the allocator (`computeDetumbleTorque`,
the cross-product desaturation law below) have to bake the compensating
negation in themselves. Getting this backwards was a real bug found during
development (see "Detumble/Desaturation" below).

---

## Detumble

Two interchangeable laws, selected by `detumbleActuator`:

**Reaction wheels** (`computeDetumbleTorque`): pure rate damping,
`torqueCommand = +Kd * rate` (the `+` sign is the wheel-convention
compensation described above; the physically-damping torque is `-Kd*rate`).
`Kd` is derived from bus inertia at `configure()`, targeting an ~8s
damping settle (`omega_n = 4/8`, `Kd = 2*I*omega_n`).

**Magnetorquers / B-dot** (`computeBdotDipoleCommand`): the classic law,
`m = -k * dB/dt` (Wisniewski's B-dot detumbling — the standard
magnetics-only technique real cubesats use right after deployment, before
wheels are trusted). `dB/dt` is finite-differenced from consecutive valid
magnetometer samples.

**Gain derivation** (`configure()`): targets near-max dipole moment at a
*representative ongoing* tumble (0.3 rad/s), not the initial deployment
peak (~1 rad/s) — the fast initial phase self-corrects regardless of exact
saturation; the actuator's utilization matters far more during the slower
"final approach." Uses `E[sin(theta)] = pi/4` (expected value of the
field/rate projection factor for a uniformly random relative orientation)
rather than the worst-case 1.0 — using the worst case as the tuning target
left rods sitting at ~15-30% saturation for most of a detumble (confirmed
via a headless sweep during development).

```
representativeDbDt = 0.3 rad/s * 30e-6 T * (pi/4)
bdotGain = maxMomentAm2 / representativeDbDt
```

---

## Momentum Desaturation (`updateDesaturation`)

Reaction wheels are purely internal actuators — they redistribute momentum
between body and wheel, never remove it from the system. Only an
*external* torque (the magnetorquers) can actually unload momentum. Runs
concurrently with whatever pointing mode is active (not a `PointingMode`
itself), using the classic cross-product law (Stickler & Alfriend):

```
m = (k / |B|^2) * headroom * (H_wheel x B)
```

where `H_wheel = sum(wheelInertia_i * speed_i * spinAxis_i)` is total
wheel angular momentum in body frame.

**Sign** (found empirically, see the "Sign convention" note above):
textbook presentations write `m = -(k/|B|^2)(h x B)`, assuming the
actuator applies `m x B` directly to the body. This sim's wheel reaction
convention doesn't — working through the Newton's-third-law negation flips
which sign actually drains `h`. The textbook sign pumped wheels to 100%
saturation and drove pointing error to 170° in a headless test; this sign
drains the wheels while holding pointing steady.

**Headroom scaling**: `headroom = clamp(1 - maxWheelSat, 0.05, 1.0)`. The
external torque this law creates has to be *absorbed* by the same wheels
being desaturated — pushing full-strength at 90% saturation left no spare
torque to react with, destabilizing pointing (170° error, confirmed
empirically); scaling by remaining headroom lets desat push hard when
there's room and automatically back off as it disappears.

**Auto-trigger**: starts when `maxWheelSat >= desatTriggerSaturation`
(default 0.8), stops at the lower `desatStopSaturation` (default 0.3) —
hysteresis, so it doesn't chatter on/off right at the trigger boundary.

**Gain derivation**: targets near-max dipole moment against a single wheel
at its own max momentum in a representative LEO field, with a `2x`
"stable speed margin." A headless gain sweep at the worst-headroom
starting point (90% saturation) showed the stability margin becomes noisy
above ~3x; 2x stayed clearly stable across every seed tested while still
desaturating meaningfully faster than 1x.

---

## FDIR / Mode Manager

`FDIR::evaluate()` runs every `step()` cycle, after the EKF correct step
(so it has a fresh `attitudeUncertaintyDeg`) and before guidance/control
(so its output can override what they compute against). See `src/FDIR.h`.

**Detected conditions** (bitmask, more than one can be active):

| Fault | Condition |
|---|---|
| `WHEEL_AUTHORITY_LOST` | fewer than `minHealthyWheels` (default 3) wheels healthy — the 4-wheel pyramid needs >=3 non-degenerate spin axes to span all 3 body axes; 2 arbitrary vectors can only span a plane |
| `ATTITUDE_UNCERTAIN` | `attitudeUncertaintyDeg > attitudeUncertaintyTriggerDeg` (default 5°), **sustained** for `attitudeUncertaintySustainedS` (default 5s) — not instantaneous, so one noisy cycle/dropped frame can't trip it |
| `EXCESS_RATE` | `|rateBody| > excessRateRadS` (default 2 rad/s) — outside the attitude controllers' tuned envelope |
| `LOW_BATTERY` | `batterySoc < lowBatterySocTrigger` (default 0.2) — instantaneous; SOC doesn't jitter like a sensor reading, no sustain timer needed |

**Latching**: once tripped, a fault stays active even if its condition
clears on its own, until `clearLatchedFaults()` (a ground command)
explicitly acknowledges it. Real FDIR does this deliberately — a wheel
that recovers on its own doesn't mean ops should stop knowing it failed.

**Mode override**: while any fault is active and `fdir.enabled`,
`effectiveMode` (what guidance/control actually execute) replaces the
commanded `mode`:

- `EXCESS_RATE` active → `DETUMBLE`, regardless of what else tripped — a
  rate outside the controller's envelope needs damping before any
  attitude-hold mode can converge cleanly.
- Otherwise → `SUN_POINTING` — the closest thing this project models to a
  real spacecraft safe mode: stable, doesn't depend on wheel count, more
  tolerant of a noisy attitude estimate than fine pointing, and (for
  `LOW_BATTERY` specifically) *literally* the physically correct response
  since it maximizes solar generation, not just a fallback choice.

`fdir.enabled = false` inhibits *acting* on faults (autonomy off) without
disabling *detection*, matching real ground-commandable autonomy inhibits
used during commissioning/testing.

---

## EPS (Electrical Power Subsystem)

**Generation** (`SolarPanel::sample`, in `spacecraft-dynamics-sim`):
standard flat-panel cosine law,

```
powerW = solarFluxWm2 * areaM2 * efficiency * max(0, cos(incidenceAngle))
```

clamped to zero (not negative) once the sun is behind the panel. One
panel per body face (6 total, 0.01 m² each — the cubesat's own 10x10cm
face), `efficiency = 0.28` (representative triple-junction cubesat cell).
`solarFluxWm2 = 1361` (solar constant at 1 AU). **Assumption**: no orbital
eclipse model exists in this sim (no orbital position/time is simulated at
all), so there's no shadow term — the sun is always "up" from wherever
`sunPosition` currently is.

**Storage** (`Battery`, in `spacecraft-dynamics-sim`): Coulomb-counting —
integrates net power (W) over `dt` (s) directly into energy (J), clamped
to `[0, capacityJ]`:

```
energyJ = clamp(energyJ + netPowerW * dt, 0, capacityJ)
stateOfCharge = energyJ / capacityJ
voltage = minVoltage + (maxVoltage - minVoltage) * stateOfCharge
```

**Assumption**: voltage is a straight linear function of state of charge.
Real Li-ion discharge curves are flatter in the middle and steeper at the
ends; this is the honest equivalent without modeling real cell chemistry —
good enough to show "voltage sags as the battery depletes," which is the
property anything reacting to it (FDIR, a UI panel) actually needs.

**Consumption** (harness-computed each FSW cycle, `satellite_adcs_sim.cpp`):
fixed loads (OBC baseline + every always-on sensor) plus effort-proportional
actuator loads, from the commands `step()` just issued:

```
drawW = POWER_OBC_BASELINE_W + POWER_IMU_W + POWER_MAGNETOMETER_W
      + POWER_STAR_TRACKER_W + POWER_SUN_SENSOR_W
      + sum_wheels( WHEEL_IDLE_POWER_W + |torqueNm * speedRadS| / WHEEL_MOTOR_EFFICIENCY )
      + sum_torquers( TORQUER_IDLE_POWER_W + |momentAm2| * TORQUER_POWER_PER_AM2_W )
```

**Assumption**: magnetorquer power is linear in commanded moment (a real
rod's resistive loss is `I^2*R`, i.e. quadratic) — a deliberate
simplification since this project has no basis for a specific coil
resistance value, and linear is close enough at these scales. Wheel power
follows mechanical power (`torque * speed`) over an assumed motor
efficiency (0.6) — this is the one term that's genuinely effort-proportional
rather than a flat idle number, matching how a real motor's draw depends
on what it's actually being asked to do.

Net power (`genW - drawW`) integrates into the battery every cycle,
computed with the *previous* cycle's SOC as `FSWInputs.power.batterySoc`
(the same "read before this cycle's effects" relationship
`wheelTelemetry[i].speedRadS` already has with the wheel commands about to
be issued).
