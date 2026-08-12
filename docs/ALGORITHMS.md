# Algorithms, Math, and Model Assumptions

This is the standing reference for the equations, algorithms, and modeling
assumptions behind this project's flight software and physical models. Code
comments explain _why_ a specific line does what it does; this document is
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

- **World frame is ECI**: Earth's center is the world origin, and
  `RigidBody::position` (and everything derived from it — `target`,
  `sunPosition`, `ambientFieldWorld`, `FSWInputs.spacecraftPositionWorld`)
  is expressed in real meters in that frame. `sat.body->position` is
  bridged from the real orbital state every frame (see "Orbital
  Mechanics" below) — the satellite genuinely moves through real ECI
  space; there is no separate small "scene" frame or visualization-only
  scale factor. `RigidBody::orientation` (a world-frame quaternion)
  relates world to body frame, via `world_vec = orientation * body_vec`
  (and `body_vec = inverse(orientation) * world_vec`). Body +Z is the
  payload/pointing axis every guidance mode aims (see "Guidance" below);
  body -Z is the star tracker boresight, deliberately opposite it.
- **Body frame**: fixed to the spacecraft, origin at its center of mass.
  Sensor readings, actuator axes, and the EKF state are all in this frame.

FSW itself never knows any of this is ECI specifically — `ADCS`/`FDIR`
only ever see the plain `FSWInputs`/`ADCS` fields (`spacecraftPositionWorld`,
`sunPosition`, ...) through the ordinary hardware-abstraction boundary
(see below), the same as they would against a HIL rig or real flight
hardware. "World frame is ECI" is a fact about what the _harness_ feeds
them, not something FSW code depends on or could tell from the inside.

## Hardware-Abstraction Boundary

`ADCS`/`FDIR`/`FlightSoftware` (the flight software) never reference
`RigidBody`, `PhysicsWorld`, or any sensor/actuator simulation type — see
`src/fsw/FlightTypes.h`. `FlightSoftware::step()` — which owns `ADCS` and
`FDIR` as peers and is this project's actual cyclic FSW entry point (see
"`FlightSoftware` / `SystemMode`" below) — is a pure function of
`(internal state, FSWInputs, dt) -> FSWOutputs`, same as
`ADCS::step()`/`control()`/`updateEstimator()` themselves. There is
deliberately no interface/HAL type crossing into `src/fsw/` at all — the
one place simulated hardware is translated to/from this plain-data
contract is `Cubesat::sampleSensors()`/`applyActuatorCommands()`
(`src/core/Cubesat.h`/`.cpp`), called by `main()` immediately before/after
`flightSoftware.step()`. This is the seam a HIL rig or real flight
hardware would eventually replace: a HIL harness would build `FSWInputs`
from real sensors and apply the returned `FSWOutputs` to real actuators
the same way `Cubesat`'s two methods do here, without `FlightSoftware`/
`ADCS`/`FDIR` changing at all. It's also why `tests/` can build FSW logic
against nothing but `glm` (see `tests/CMakeLists.txt`).

---

## Spacecraft Structure: Composite Mirror + Bus Mass/Inertia

A Reflect-Orbital-style mission concept: an 18m x 18m deployable mirror
(the `RigidBody` itself, `src/core/Cubesat.cpp`'s `buildCubesatPyramid()`)
with a much smaller bus at its center. The mirror plate and bus core are
coaxially centered on the same point (a deployable-membrane layout like
IKAROS's real solar sail — boom-mounted membrane, bus at the hub), so the
composite inertia is a direct sum of each element's own inertia about the
shared center — no parallel-axis term is needed.

**Mirror**: 18m x 18m x 5mm box, areal density 0.1 kg/m² (~2x IKAROS's
real ~48 g/m² sail-only figure, accounting for a mirror needing better
optical/structural quality than a pure photon-pressure sail) → ~32.4 kg
raw, rounded up to 35 kg for supporting boom/structure margin —
representative, not a real hardware spec (none exists at this scale).
Thin-plate box inertia about its own center (thickness term negligible
against the 18m span):

```
Ix = Iy = m(L² + t²)/12 ≈ mL²/12
Iz = m(Lx² + Ly²)/12 = mL²/6   (square plate, Lx = Ly = L)
```

**Bus**: 0.5m cube core, 40 kg representative small-bus mass (avionics,
mirror-deployment mechanism). The mission concept's "~2.5m envelope with
panels deployed" is the deployed solar-panel span, not the structural
core — those panels are thin/low-mass, and their inertia contribution is
folded into the bus's own margin rather than modeled as a separate
element. Standard cube inertia, `I = ms²/6` (all axes).

**Composite** (sum, both about the shared center): total mass 75 kg,
`Ixx = Iyy ≈ 946.7 kg·m²`, `Izz ≈ 1891.7 kg·m²` — confirmed by direct
computation in a headless check against `buildCubesatPyramid()`'s actual
output. Applied via `RigidBody::setMass()`/`setInertiaTensor()`
(`spacecraft-dynamics-sim`'s engine API for overriding the shape-derived
mass/inertia with a more accurate composite value), not just a resized
bounding box.

This is a ~450,000-850,000x jump from the project's original 1U-cubesat
inertia (~0.002 kg·m²), which in turn required resizing the wheel/
magnetorquer actuators so the spacecraft retains meaningful angular
control authority. Real reaction wheels are sized to the _bus's_
mass/power/volume budget (a smallsat buys an off-the-shelf wheel that
fits its bus, it doesn't size a custom wheel to the payload's inertia),
so realistic sizing here means real hardware in the 50-150 kg smallsat
class, not the payload's own huge inertia:

- **Wheel `maxTorqueNm`**: 0.025 Nm, **`wheelInertia`**: 1.6e-3 kg·m²
  (~1.0 Nms momentum capacity at `maxSpeedRadS`) — matching real
  smallsat-class reaction wheels (e.g. Blue Canyon RWp500 / Sinclair
  RW-1.0 class).
- **Magnetorquer `maxMomentAm2`**: 15.0 A·m² — matching a real
  torque-rod product in this bus class (e.g. ZARM Technik MT15-1).

This is _smaller_ than an earlier placeholder value used during initial
verification (1.0 Nm / 20 A·m², explicitly flagged then as "not a real
spec"), and much smaller than what would be needed to hit the project's
original 1U-cubesat-scale settling-time targets at this inertia — see
"Torque-aware auto-tune" under Attitude Control below for how that's
reconciled.

See "Torque-aware auto-tune" (under Attitude Control) for how
`ADCS::retuneForMode()` keeps the settling time each controller actually
chases consistent with what these actuators can deliver.

---

## Orbital Mechanics

The satellite's real orbital position/velocity is truth-propagated by
`spacecraft-dynamics-sim`'s `rigidbody/orbit/` module (double precision,
`glm::dvec3`/`glm::dquat`) as part of `PhysicsWorld::step()` itself, via
the engine's **orbital mode**: `PhysicsWorld::attachCelestialSystem()` +
`setOrbitalMode()` puts a body's _translational_ state under a
`CelestialSystem`-driven RK4 propagation (two-body toward its primary,
`CelestialPerturbation` for each other body in the system), while its
_rotational_ dynamics (orientation, angular velocity, actuator
`ForceGenerator`s) keep integrating exactly like any other `RigidBody`.
`SimulationState::buildSimulationState()` (`src/core/SimulationState.h`/
`.cpp`) builds a Sun → Earth → Moon `CelestialSystem` (Earth/Moon reuse the
existing low-precision `SunModel`/`MoonModel` analytic ephemeris as their
parent-relative position) and puts the spacecraft in orbital mode around
Earth, perturbed by the Sun and Moon — the same "build once at setup" role
`buildCubesatPyramid()` plays for the spacecraft itself. `ADCS` never sees
any of this directly — it stays hardware-abstracted, only ever reading the
derived quantities below (fed to it by `Fsw::step()`, `src/core/Fsw.h`/
`.cpp`).

**One fixed-rate loop drives `SimulationState::step()` and one `Fsw` cycle
together**, in that order, once per `Config::TIME_STEP_S`
(`main()`): `sim.step(dt)` — `world.step(dt)` (propagates the Sun/Earth/
Moon hierarchy and the spacecraft's orbital-mode state, then integrates
rotational dynamics against whatever actuator commands
`Cubesat::applyActuatorCommands()` issued at the _end_ of the previous
cycle — zero-order hold, the standard discretization for a sampled control
loop), followed by refreshing `SimulationState`'s own cached eclipse/
field/sun-direction quantities from `world.isInEclipse()`/
`ambientFieldAt()`/`absolutePosition()` queries — then `fsw.step(dt)` —
ground-station target selection, `Cubesat::sampleSensors()` →
`flightSoftware.step()` → `Cubesat::applyActuatorCommands()`, EPS, and
telemetry. Keeping orbit propagation and rotational dynamics inside the
same `world.step()` call (rather than two separately-scheduled systems the
harness has to keep in sync by hand) is what guarantees every actuator
command a control cycle issues actually gets integrated before the next
one runs, and is what lets `SimulationState`/`Fsw` _read_ eclipse/field/
sun-direction as `PhysicsWorld` query results instead of re-deriving them.

**Double-precision truth, bridged into `RigidBody` every FSW cycle**: a
LEO orbital radius (~6.9e6 m) leaves float32 with only meter-level
precision, and that error would compound every integration step over a
mission that can run for months — so the _integration_ itself happens in
the double-precision `orbit/` module, not through `PhysicsWorld`'s own
(float32) translational stepping. Each cycle, the harness copies the
result into `sat.body->position` as a single non-accumulating cast (see
`rigidbody/orbit/OrbitState.h`'s header comment) — this is safe precisely
_because_ it's a fresh copy from the double-precision truth every frame,
not something `PhysicsWorld` integrates on its own: `sat.body->velocity`
is deliberately never set to match real orbital velocity, so
`PhysicsWorld`'s own translational integration contributes nothing on
top of it. The body also has `groundCollisionEnabled = false` (see
`RigidBody.h` in the engine) — `PhysicsWorld`'s ground-collision
resolution assumes a literal world-Z=0 ground plane, which doesn't exist
in an Earth-centered frame where Z genuinely crosses zero every orbit.
`CentralBodyGravity` (a float32 `ForceGenerator`, also in the engine)
exists separately for shorter-duration/local-effect scenarios that don't
need this precision — a body actually integrating its own trajectory
under real gravity within `PhysicsWorld`, rather than having the
double-precision truth bridged in.

**Propagation**: RK4 integration of a 6-element `[position, velocity]`
ECI state, summing a set of pluggable force models each stage
(`rigidbody/orbit/OrbitForceModel.h`):

- **Two-body gravity**: `a = -mu * r / |r|^3` (point-mass Earth).
- **J2 perturbation**: the dominant secular effect a pure point-mass model
  misses — Earth's oblateness causes real nodal regression (the orbital
  plane precessing over time) and apsidal drift. Standard closed-form
  acceleration (Vallado eq. 9-38); no higher-order zonal harmonics.
- **Atmospheric drag** (`rigidbody/orbit/AtmosphericDrag.h`):
  `a = -0.5 * rho(alt) * Cd * (A/m) * |v_rel| * v_rel`, where `v_rel` is
  velocity relative to the atmosphere (which co-rotates with the planet:
  `v_atm = omega x r`, not inertial/ECI velocity directly). `rho(alt)` is
  a 23-layer piecewise-exponential fit to the US Standard Atmosphere 1976
  (0–1000 km, each layer's own reference density/scale height), zero
  above 1000 km — a distinct, higher-fidelity model from `UniformDrag`/
  `CentralBodyDrag`'s single fixed scale height, since this is the
  mission-duration truth propagator's own drag term, not a short-duration
  local approximation.
- **Solar radiation pressure** (`rigidbody/orbit/SolarRadiationPressure.h`):
  `a = -P(r_sun) * Cr * (A/m) * r_hat_(sun->sat)`, where
  `P(r_sun) = P_1AU * (AU/r_sun)^2` (inverse-square, same as solar flux)
  and `Cr` is the reflectivity coefficient (1.0 = perfect absorber,
  ~2.0 = perfect reflector; defaults to 1.3, typical for a mixed real
  surface). Zeroed during eclipse via the same `EclipseModel` used for EPS
  generation gating. `A/m` (`Config::SPACECRAFT_CROSS_SECTION_M2` /
  `SPACECRAFT_MASS_KG`) uses the mirror's full 18m x 18m face area against
  its 75kg composite mass (see "Spacecraft Structure" above) — at this
  ratio SRP is no longer a minor perturbation (~2.9 mN at 1 AU for a
  reflective 324 m² surface), so visibly larger SRP-driven orbital
  perturbation than the project's original 1U-cubesat scale is expected,
  not a bug.
- **Third-body gravity, Sun and Moon** (`rigidbody/orbit/ThirdBodyGravity.h`):
  `a = mu_body * (d/|d|^3 - r_body/|r_body|^3)`, where `d = r_body -
r_satellite`. The second (`-r_body/|r_body|^3`) term is the "indirect"
  correction that removes Earth's own acceleration toward the third body,
  which is what keeps the result correct in the (non-inertial-relative-to-
  the-third-body, but inertial-relative-to-Earth) ECI frame this project
  uses — without it the formula would double-count Earth's own fall
  toward the Sun/Moon as a perturbation on the satellite. Moon position
  comes from `rigidbody/orbit/MoonModel.h` (Meeus, _Astronomical
  Algorithms_ ch. 47, reduced to its dominant periodic terms — ~0.3°
  accurate, the same precision tier as `SunModel`).

All four of these perturbations are added to both the real
`orbitPropagator` and `computePredictedOrbitPath()`'s temporary one (see
harness) — the predicted-path polyline uses the same force models the
real propagator does, not a simplified stand-in, since a two-body+J2-only
prediction would visibly diverge from the real perturbed trajectory over
a full orbit.

This project's default orbit is ISS-like (500 km circular, 51.6°
inclination), specified via classical orbital elements
(`rigidbody/orbit/OrbitalElements.h`, the standard COE→RV conversion) —
the same default altitude/inclination the project's earlier kinematic
magnetic-field stand-in used, now driving a real trajectory instead of a
fake phase angle.

**Frames/time** (`rigidbody/orbit/OrbitFrames.h`, `OrbitTime.h`):
spherical Earth (not WGS84 ellipsoidal — a documented simplification, not
needed at this project's pointing/visualization precision), Julian Date
epoch handling, and standard GMST (Vallado Alg. 15) for ECEF↔ECI rotation.

**Sun direction** (`rigidbody/orbit/SunModel.h`): the standard low-precision
solar-position formula (a few arcminutes accurate near J2000, degrading
slowly further out) — nowhere near ephemeris-grade, which this project has
no need for.

**Eclipse** (`rigidbody/orbit/EclipseModel.h`): cylindrical shadow model —
in shadow if the satellite is on the night side of Earth's center _and_
within Earth's radius of the Earth-Sun line. Slightly overestimates
eclipse duration versus the true conical umbra/penumbra; adequate for
gating EPS generation (see "EPS" below), not for anything needing a
precise penumbra transition.

**Sun/Moon rendering** (harness, `main.cpp`'s draw block):
both are drawn at their real ECI positions and real physical radii, not
scaled-for-visibility stand-ins — the Sun via `adcs.sunPosition` (already
the real `SunModel::positionEci`) and `OrbitFrames::SUN_RADIUS_M`, the
Moon via a per-frame `MoonModel::positionEci` cast (`moonPositionNow`,
held while paused, same pattern as `earthRelativePositionNow`) and
`OrbitFrames::MOON_RADIUS_M`. True size at true distance for both, the
same treatment Earth already gets — no angular-diameter-at-a-fixed-
apparent-size trick. `Config::CAMERA_FAR` (2.0e11 m) is sized to clear
the real Sun distance with margin — both markers would otherwise be
silently clipped by the far plane despite their positions being computed
correctly.

Both are drawn with real textures (`resources/textures/sun.jpg`/`moon.jpg`,
falling back to a plain colored sphere via `Texture::isLoaded()` if the
asset is missing — the same graceful-degradation pattern `earthTexture`
already used) rotated by `OrbitRenderer.h`'s
`TEXTURED_SPHERE_POLE_ALIGNMENT`, the same mesh-orientation fix `drawEarth`
applies (see its own comment: VGL's Texture loader flips images
vertically on load, so without this rotation the sphere renders upside-
down). Unlike Earth, neither gets `drawEarth`'s additional meridian
correction or GMST-based spin — those exist specifically to align Earth's
texture to real ECI/geodetic longitude, and neither the Sun nor the Moon
has a real rotation/prime-meridian model in this project to align to;
`TEXTURED_SPHERE_POLE_ALIGNMENT` alone is enough to make either read as
"a normal right-side-up sphere," not a claim about their true rotational
orientation.

The Sun is drawn **unlit** (`GUI::drawTexturedSphere`'s `unlit` parameter,
new in VGL alongside this feature): a light source, not an object lit by
one, so it should render at full texture brightness from every angle
rather than being shaded/shadowed like an ordinary lit sphere would be on
its own far side. The Moon is drawn normally lit, since it has no light
of its own and should shade realistically under the same directional
light Earth does.

**Scene lighting** (harness, main render loop): VGL's single directional
light (`GUI::setLightDirection`) defaults to a fixed, arbitrary world-space
direction with no notion of where the Sun actually is — the harness sets
it to `normalize(adcs.sunPosition)` every frame, right before
`gui.beginFrame()` (the real direction from Earth's center toward the
Sun) so Earth's lit hemisphere actually corresponds to where the Sun
marker is drawn. The shader's diffuse term is
`dot(normal, normalize(lightDir))`, i.e. `lightDir` must point _toward_
the light source, not describe the Sun's position itself — using the raw
`adcs.sunPosition` vector unnormalized (or negated) here would light the
wrong hemisphere.

Ambient light (`GUI::setAmbientLight`, new in VGL alongside this feature
— previously a hardcoded `0.15` in the shader, not configurable per
scene) is set once at startup to `Config::SCENE_AMBIENT_LIGHT` (0.35):
VGL's original fixed value left Earth's night side reading as near-black,
harsher than this project's visual goal of a softer still-visible dark
side — not a claim about real earthshine/city-light brightness, purely a
readability choice.

**Global magnetic field-line visualization** (`traceDipoleFieldLines`/
`drawMagneticFieldLines`, harness — distinct from `drawMagneticField`'s
single arrow at the satellite showing the locally-sampled vector that
actually drives magnetorquer/magnetometer FSW): traces closed dipole
field-line loops connecting Earth's magnetic poles, in the same style as
a textbook/magnetosphere field-line diagram. Seeded on a colatitude/
azimuth grid near both poles (measured from `CentralBodyMagneticField`'s
`rotationAxisWorld`, an approximation of the true ~11°-tilted dipole axis
that's exact enough for seed _placement_ — the traced geometry itself is
exact, since every step samples the real field), each line is integrated
as an RK4 arclength streamline (`dP/ds = sign * B(P)/|B(P)|`, not a
time-stepped trajectory) until it either closes back onto the surface or
crosses a safety-radius cutoff (guards near-axis seeds, whose loops are
much larger). `sign` is chosen per-seed so the first step moves radially
outward, matching how a real dipole loop leaves the surface at one pole
and curves back down to the other. Traced once at startup, not per frame,
since this model's dipole is fixed-inertial (no time dependence — see
`CentralBodyMagneticField`'s own header comment); drawn every frame in
two tones by seed hemisphere (cyan/amber) so which pole a given loop
connects to reads at a glance.

**Reaction-wheel/magnetorquer mount-position precision**
(`drawReactionWheels`/`drawMagnetorquers`, harness): these read the
public `mountPositionBody`/`spinAxisBody`/`axisBody` fields directly and
apply `SATELLITE_VISUAL_SCALE` to the (small, ~0.03 m) mount offset
_before_ rotating and adding it to the satellite's real (~6.9e6 m)
position — not
`wheel->getWorldMountPosition(*sat)` (which computes `body.position +
body.orientation * mountOffset` internally, in float32, before the
harness ever sees the result). Scaling after that internal addition
can't recover precision already lost to the same catastrophic-
cancellation pattern described above for `adcs.sunPosition`/ground-station
targeting (below) — the general fix is always to scale the small offset
up to a normal-sized number _first_, then add it to the large base value,
never the reverse.

**Orbital elements / geodetic conversion, for the Orbit tab**
(`rigidbody/orbit/OrbitalElements.h`'s `fromState()`, `OrbitFrames.h`'s
`ecefToGeodetic`/`eciToGeodeticDeg`): the standard RV2COE algorithm
(angular momentum/node/eccentricity vectors, vis-viva for semi-major
axis) recovers classical elements (altitude, eccentricity, inclination,
RAAN, argument of periapsis, true anomaly, period, apogee/perigee) from
`orbitState`'s _real, propagated_ position/velocity — displayed elements
drift from the initial commanded values as J2 acts on the orbit (nodal
regression, apsidal drift — see "Propagation" above), same as a real
spacecraft's actual orbit does. The ground-track minimap and the 3D
ground-footprint circle (main.cpp's `drawGroundTrackMinimap`/
`drawGroundFootprint`, structurally ported from `constellation-sim`'s
`SatelliteRenderer`) both derive from this same real state — the
footprint's coverage half-angle uses the standard elevation-angle
ground-coverage formula (`rho = pi/2 - minElevation - eta`,
`sin(eta) = R*cos(minElevation)/r`), with `minElevation =
Config::FOOTPRINT_MIN_ELEVATION_DEG` (0°, horizon-limited, by default) --
the same threshold ground-station target selection uses (below), so "is
this station in the footprint circle" and "is this station a valid
targeting candidate" never disagree.

**What the harness derives from this each cycle**, and feeds across the
hardware-abstraction boundary as plain `FSWInputs`/`ADCS` fields (see
"Coordinate Frames" above — FSW itself just sees these as ordinary
vectors, the same as it would from a HIL rig or real hardware):

- `FSWInputs.spacecraftPositionWorld` — the real orbital position itself
  (`sat.body->position`, bridged from `orbitState` as described above),
  used directly by `NADIR`/`TARGET`/`SUN_POINTING`/`REFLECT` guidance
  (see "Guidance" below).
- `adcs.ambientFieldWorld` — sampled from `CentralBodyMagneticField` (the
  engine's tilted-dipole model, same formula the old kinematic stand-in
  used) at the real position, instead of a fake orbital phase.
- `adcs.sunPosition` — the real Sun position (`SunModel::positionEci`),
  not an arbitrary nearby offset. This matters numerically, not just for
  correctness: with `spacecraftPositionWorld` now at real (~6.9e6 m)
  scale, encoding the sun _direction_ as a small offset from it (the
  previous "2 units away" convention) would suffer catastrophic
  cancellation once `sunPosition - spacecraftPositionWorld` is computed
  back out in float32 — the small offset gets rounded away against the
  much larger base position. The real Sun position (~1.5e11 m) doesn't
  have this problem: it's the _dominant_ term in that subtraction, not a
  small perturbation of one.
- EPS generation gating — see "EPS" below.

**Ground-station targeting** (`GroundStations.h/.cpp`, harness): `TARGET`/
`SLEW`/`FINE_POINTING`/`REFLECT` all aim at `adcs.target`, which the
harness sets every simulated frame to the real ECI position of the
closest ground station currently _within the satellite's footprint_
(elevation ≥ `Config::GROUND_STATION_MIN_ELEVATION_DEG`, 10° — a typical
real minimum usable elevation, distinct from
`Config::FOOTPRINT_MIN_ELEVATION_DEG`'s 0°/horizon-limited pure
_geometric coverage_ circle, see its own comment in `Config.h`) — not an
arbitrary point, and not a fixed one. `GROUND_STATIONS` is a small
compiled list of major US cities' geodetic lat/lon, standing in for real
ground-segment sites (see README.md's Scope section on this project's
boundary). Each station's real ECI position is recomputed every call
(`groundStationPositionEci`, `OrbitFrames::geodeticToECEF` composed with
`ecefToECI` at the current Greenwich sidereal angle) rather than cached,
since a fixed ECEF site sweeps through ECI as Earth rotates underneath it
— the same rotation that makes `drawEarth`'s globe visibly spin.

`selectClosestGroundStation` picks the closest (by straight-line slant
range to the satellite) station among those meeting the minimum-elevation
threshold (`OrbitFrames::elevationAngleRad`, the same formula
`drawGroundFootprint`'s coverage circle and pass prediction below both
use), or returns `nullptr` if none currently do — a station outside the
footprint is not a viable target at all, so there is deliberately no
"closest regardless of visibility" fallback.

When no station is reachable, the harness sets `adcs.targetValid = false`
rather than spoofing `adcs.target` itself — the typical real-ADCS
convention for "this mode's reference isn't available right now" is a
guidance-level degradation, not the caller faking a different target
position. `ADCS::computeGuidance()` reads `targetValid`: whenever the
commanded/effective mode is `TARGET`/`SLEW`/`FINE_POINTING`/`REFLECT` and
`targetValid` is false, guidance substitutes the same direction
`SUN_POINTING` itself would compute
(`normalize(sunPosition - spacecraftPositionWorld)`) for that cycle only —
`REFLECT`'s own bisector math is skipped entirely in this case (no target
means no bisector to compute; pointing the mirror straight at the sun is
the sensible fallback). This never touches `adcs.mode`/`effectiveMode`
themselves (so a manually-commanded mode via the FSW tab/`[1]`-`[7]` keys
is never fought by this logic) or controller gain selection — only which
direction guidance aims at. `adcs.target` is simply left stale (harmless
— unread while `!targetValid`) when no station is selected. The
ground-station case is at real Earth-surface scale
(`OrbitFrames::EARTH_RADIUS_M`), the same reason the project's earlier
arbitrary-surface-point target was placed there: comparable in magnitude
to `spacecraftPositionWorld` (a well-conditioned
`target - spacecraftPositionWorld` subtraction) rather than negligible
next to it (catastrophic cancellation) — the sun-relative fallback
doesn't have this problem either, `sunPosition` being the _dominant_ term
in that same subtraction at ~1.5e11 m.

The harness only resets the attitude controller's integral windup
(`adcs.resetController()`) when _what's selected_ actually changes — a
different ground station, or `targetValid` flipping — not every frame:
`adcs.target`'s position updates continuously as the selected station's
ECI position sweeps with Earth's rotation, which the controller should
track smoothly, not react to as a discrete retarget each cycle.

**Ground-station pass prediction** (`GroundStations.h/.cpp`'s
`predictGroundStationPasses`, `GroundStationsPanel.h/.cpp`): the Ground
Stations tab's contact schedule — a first step toward comms-link
simulation, per this project's own trajectory (see README.md's Scope
section). For each of the next `Config::PASS_PREDICTION_LOOKAHEAD_S`
(24h) of simulated time, a _copy_ of the real orbital state is propagated
forward in fixed `Config::PASS_PREDICTION_STEP_S` (15s) increments — the
same force models as the real propagator and `computePredictedOrbitPath`
(two-body, J2, drag, SRP, Sun/Moon third-body), for consistency — sampling
every `GROUND_STATIONS` entry's elevation at each step. A per-station
state machine detects AOS (elevation crosses above
`Config::GROUND_STATION_MIN_ELEVATION_DEG`) and LOS (crosses back below),
tracking the peak elevation reached in between; a pass still open when
the lookahead window ends is recorded with LOS at the window's own end
instant, not a fabricated later time. AOS/LOS timing resolution is
therefore ±`PASS_PREDICTION_STEP_S` (a coarser step trades timing
precision for prediction speed) — displayed at whole-second resolution in
the UI, since implying sub-step precision would overstate what a 15s-step
search actually resolved.

Refreshed on a **real wall-clock** timer (`Config::PASS_PREDICTION_REFRESH_S`,
30s), not simulated time like `computePredictedOrbitPath`'s own refresh —
tying a ~5760-step search's cadence to `simDt` would make it run _more_
often, not less, the faster `SimControls::timeScale` is turned up, the
opposite of what keeping the frame rate steady at high time-scale needs.

AOS/LOS clock times are converted from Julian Date via the engine's
`OrbitTime::calendarDate` (the inverse of `julianDate` — Meeus's standard
Julian-Date-to-Gregorian-calendar algorithm), added to
`spacecraft-dynamics-sim` alongside this feature since no forward-only
JD handling previously needed to go the other way.

**Deliberately not modeled**: max data rate, expected data volume, link
margin, weather, and scheduling priority — a real ground-station pass
report's other usual fields — are shown in the Ground Stations tab's
pass-detail card as explicitly "not yet modeled" rather than a plausible-
looking fabricated number, per this project's standing rule against
fabricating simulation results. They need a link-budget model (antenna
gain/pointing loss, path loss, receiver noise figure), a weather model,
and a scheduling/conflict-resolution concept respectively, none of which
exist yet — the geometric quantities (`GroundStationPass`'s actual
fields) are the honest subset this project can compute today from real
orbital mechanics.

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

| Mode                                | `pointDir`                                                                                                                      |
| ----------------------------------- | ------------------------------------------------------------------------------------------------------------------------------- |
| `NADIR`                             | `-normalize(spacecraftPosition)` — world frame is ECI (see "Coordinate Frames" above), so this is a real Earth-center direction |
| `SUN_POINTING`                      | `normalize(sunPosition - spacecraftPosition)`                                                                                   |
| `TARGET` / `SLEW` / `FINE_POINTING` | `normalize(target - spacecraftPosition)`                                                                                        |
| `REFLECT`                           | see below                                                                                                                       |
| `DETUMBLE`                          | none — bypasses guidance entirely, see "Detumble"                                                                               |

`REFLECT`'s `target` is still whatever `adcs.target` currently holds
(ground-station auto-tracking, same as `TARGET`/`SLEW`/`FINE_POINTING`).
`src/core/SolarFarms.h`'s `SOLAR_FARMS` (real-world utility-scale solar
farm sites, for a Reflect-Orbital-style mission concept) is data +
globe-marker visualization only — deliberately not wired into `target`
auto-selection here. Auto-tracking a solar farm the way ground stations
are tracked (accounting for e.g. the site needing to be in dusk/dawn
shadow while the spacecraft itself stays sunlit, the actual geometry a
real orbital-mirror mission needs) is a natural follow-up, not implemented.

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
6x6 covariance describes uncertainty in a small-angle _error_ around them.
This is the standard approach for attitude EKFs since quaternions have no
flat vector space to naively apply a linear Kalman filter to.

Covariance is stored as four `glm::mat3` blocks (`covAA`, `covAB`, `covBB`)
instead of one 6x6 matrix, since GLM has no fixed 6x6 type — `covBA` is
never stored separately (`P` is symmetric by construction, every update
preserves that).

**Known issue (tracked separately, not fixed here)**: `propagateEstimator()`/
`correctEstimator()` update `covAA`/`covAB`/`covBB` with a raw assignment
each cycle — no symmetrization, no positive-semi-definiteness safeguard
(no Joseph-form update), and all in `float` (32-bit), not `double`.
Headless testing (a spacecraft held in continuous, well-converged
SUN_POINTING for several thousand simulated seconds — tens of thousands
of propagate/correct cycles) reproducibly hit a hard NaN in `covAA`
(propagating instantly into `estimatedAttitude`, body rate, and every
wheel state the same cycle), consistent with float32 accumulation error
eventually breaking `covAA`'s positive-definiteness until `correctEstimator()`'s
`glm::inverse(S)` (`S = covAA + R*I`, no conditioning check) blows up.
Reproduced across multiple independent runs, always after a long,
tightly-converged hold, never during a large initial-error transient —
this wasn't caught earlier because no prior test in this project ran the
EKF continuously this long. A real fix (covariance symmetrization each
update, and/or a small positive floor, and/or switching this state to
double precision) is a genuine follow-up, not something to patch as a
side effect of an unrelated change.

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

### Torque-aware auto-tune

Each controller's `autoTune()` independently derives `omega_n = 4 /
(dampingRatio * settlingTime)` from the requested settling time, then
sizes gains off it (`Kp = I*omega_n^2`, `Kd = 2*dampingRatio*I*omega_n`,
or the LQR-equivalent Q/R back-solve). At high inertia, a settling-time
target alone can demand torque the actuators don't have — pure gain
scaling from inertia doesn't account for actuator authority at all.
`autoTune()` now also takes `maxControlTorqueNm` and caps `omega_n`
against it: for a worst-case (antipodal, `theta_max = pi`) attitude error
at zero rate, the peak commanded torque is `Kp * theta_max =
I*omega_n^2*pi`; capping that at the budget gives

```
omega_n = min(4 / (dampingRatio * settlingTime),
              sqrt(maxControlTorqueNm / (I * pi)))
```

If the requested settling time is already achievable within budget this
is a no-op; otherwise the _achieved_ settling time is longer than
requested — self-derived from real actuator authority rather than a
second hardcoded constant to keep in sync by hand. PID/Cascaded use the
average diagonal `I = (Ixx+Iyy+Izz)/3` (same as their existing gain
derivation); LQR applies this per-axis (`I = inertiaTensor[i][i]`,
matching its own existing per-axis Kp/Kd solve) — for this project's
mirror spacecraft, Izz (1891.7) is 2x Ixx/Iyy (946.7), so the Z axis gets
its own, looser ceiling.

`ADCS::retuneForMode()` passes `hw_.wheels[0].maxTorqueNm` (representative
single-actuator value, the same convention `bdotGain`/`desatGain` already
use for `hw_.torquers[0]`/`hw_.wheels[0]`) scaled by a **0.15x margin**,
not the raw value. The static Kp-only worst-case estimate above doesn't
capture the `Kd*rate` term's own torque contribution once the body is
actually moving — under `PID` in particular, which (unlike Cascaded/LQR)
has no intermediate rate-command saturation stage to bound that term, so
it can approach the same order as the Kp term during a real transient.
Headless testing (SUN_POINTING from 90° initial error, this spacecraft's
actual composite inertia and realistic wheel torque) confirmed the full
(1.0x) budget sustains an undamped limit cycle rather than converging,
and 0.35x converges but with a visibly under-damped, regrowing
oscillation; 0.15x converges smoothly and holds a stable small error for
the several-thousand-second window verified, cross-checked across
several random IMU-noise seeds. (An unrelated, pre-existing EKF
covariance-conditioning bug — see "Attitude Estimation" — currently
limits how much longer a single continuous run can go before an
unrelated NaN; it is independent of this margin's value and is tracked
separately, not fixed here.)

**Sign convention** (load-bearing, found by empirical testing, not just
derivation): `ReactionWheel`'s simulated reaction dynamics apply the
_negative_ of whatever torque is commanded (Newton's-third-law reaction —
spinning the wheel one way reacts the bus the other way). Every control
law here computes the torque it wants applied _to the bus_, and
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
_representative ongoing_ tumble (0.3 rad/s), not the initial deployment
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

**Automatic entry/exit** (`checkAutoDetumbleEntry`, called from
`updateEstimator()`): `ADCS` itself, not `FDIR`, autonomously commands
`mode = DETUMBLE` when bias-corrected `|rate|` exceeds
`detumbleEntryRateRadS` (1.3 rad/s) while not already in `DETUMBLE`, and
reverts to `SUN_POINTING` once an _auto-entered_ `DETUMBLE`'s rate damps
below the lower, hysteresis `detumbleExitRateRadS` (0.1 rad/s) — a
manually-commanded `DETUMBLE` is never auto-exited (tracked via an
internal "was this auto-entered" flag). Runs before `FDIR::evaluate()`
each cycle (see `FlightSoftware::step()`), so a same-cycle mode change is
visible to FDIR's own `commandedMode` input, not delayed a cycle.

This is deliberately distinct from FDIR's `EXCESS_RATE` fault (see "FDIR
/ Mode Manager" below): that fault is a higher-threshold (2.0 rad/s)
anomaly backstop that _latches_ until a ground command explicitly clears
it, appropriate for a genuinely off-nominal condition. Routine post-
deployment tumbling is an expected mission phase, not an anomaly, so this
mechanism is non-latching and self-clearing at a lower, "can't usefully
point yet" threshold instead. `detumbleEntryRateRadS`'s value has to
clear `SLEW`'s own commanded-rate cap (`ModeTuning::omega_max`, 1.0
rad/s) with real margin — an aggressive slew genuinely pins body rate
near that cap (see `Controllers.cpp`), not just a theoretical ceiling, so
a lower entry threshold would false-trigger DETUMBLE mid-maneuver
(verified directly in `tests/test_detumble.cpp` against that exact
boundary).

**Automatic actuator selection** (`DetumbleActuator::AUTO`, the default):
each cycle, `ADCS::control()` computes peak wheel saturation
(`max(|speed_i / maxSpeed_i|)`, the same ratio `updateDesaturation()`
already computes for its own thresholds) and picks reaction wheels while
that stays under `detumbleWheelSaturationBudget` (0.3) — fast/precise,
and wheels typically start empty right after deployment — handing off to
magnetorquers once that budget's used up, for the rest of the detumble.
The resolved choice is published as `activeDetumbleActuator` (mirrors
`detumbleActuator` directly when it isn't `AUTO`), the same "commanded X,
actually flying Y" telemetry pattern `effectiveMode` already uses.
`REACTION_WHEELS`/`MAGNETORQUERS_BDOT` remain selectable as explicit
manual overrides.

**Deployment** (`SimulationState::buildSimulationState()`, harness):
the spacecraft starts with a real nonzero angular velocity — a per-axis
uniform kick over `[-Config::DEPLOYMENT_TUMBLE_RATE_RAD_S,
+Config::DEPLOYMENT_TUMBLE_RATE_RAD_S]` (1.6 rad/s), the same shape as
the manual "Kick into random tumble" button, applied once at startup
instead of on a button press. `1.6` is chosen so the expected 3-axis
vector magnitude (`E[|v|] ≈ R` for per-axis `Uniform(-R,R)`) sits above
`detumbleEntryRateRadS` with real margin, while staying a typical case
well under FDIR's `excessRateRadS` (a worst-case draw near `R*sqrt(3) ≈
2.77` is a rare tail, not the common case).

---

## Momentum Desaturation (`updateDesaturation`)

Reaction wheels are purely internal actuators — they redistribute momentum
between body and wheel, never remove it from the system. Only an
_external_ torque (the magnetorquers) can actually unload momentum. Runs
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
external torque this law creates has to be _absorbed_ by the same wheels
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

`FDIR` is a peer module of `ADCS`, not nested inside it — both are owned
by `FlightSoftware` (`src/fsw/FlightSoftware.h`/`.cpp`), this project's
actual top-level cyclic entry point (see "Hardware-Abstraction Boundary"
above and "`FlightSoftware` / `SystemMode`" below). This matches real
flight-software convention (NASA cFS, JPL's F´, most smallsat stacks):
fault/health monitoring is typically its own module sitting above or
beside the subsystems it watches, with authority over system-wide mode,
rather than being embedded inside and invoked by one subsystem's own
control loop.

`FDIR::evaluate()` runs every `FlightSoftware::step()` cycle, called
directly by that function — after `ADCS::updateEstimator()` (the EKF
predict+correct, so `evaluate()` has a fresh `attitudeUncertaintyDeg` and
bias-corrected rate to read) and before `ADCS::control()` (so its output,
written into `ADCS::effectiveMode`, is what guidance/control actually
compute against that cycle). See `src/fsw/FDIR.h` for `FDIR`'s own public
API, which this restructure left unchanged — only _who calls_ `evaluate()`
moved.

**Detected conditions** (bitmask, more than one can be active):

| Fault                  | Condition                                                                                                                                                                                                 |
| ---------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `WHEEL_AUTHORITY_LOST` | fewer than `minHealthyWheels` (default 3) wheels healthy — the 4-wheel pyramid needs >=3 non-degenerate spin axes to span all 3 body axes; 2 arbitrary vectors can only span a plane                      |
| `ATTITUDE_UNCERTAIN`   | `attitudeUncertaintyDeg > attitudeUncertaintyTriggerDeg` (default 5°), **sustained** for `attitudeUncertaintySustainedS` (default 5s) — not instantaneous, so one noisy cycle/dropped frame can't trip it |
| `EXCESS_RATE`          | `                                                                                                                                                                                                         | rateBody | > excessRateRadS`(default 2 rad/s) — outside the attitude controllers' tuned envelope. Higher-threshold, latched anomaly backstop — distinct from`ADCS`'s own lower-threshold, non-latching automatic detumble entry (see "Detumble" above), which handles routine post-deployment tumbling before it would ever reach this |
| `LOW_BATTERY`          | `batterySoc < lowBatterySocTrigger` (default 0.2) — instantaneous; SOC doesn't jitter like a sensor reading, no sustain timer needed                                                                      |

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
  `LOW_BATTERY` specifically) _literally_ the physically correct response
  since it maximizes solar generation, not just a fallback choice.

`fdir.enabled = false` inhibits _acting_ on faults (autonomy off) without
disabling _detection_, matching real ground-commandable autonomy inhibits
used during commissioning/testing.

---

## `FlightSoftware` / `SystemMode`

`FlightSoftware` (`src/fsw/FlightSoftware.h`/`.cpp`) is this project's
actual cyclic FSW entry point: sensor data in (`FSWInputs`), actuator
commands out (`FSWOutputs`), once per call. It owns `ADCS` and `FDIR` as
peers and runs them in the same "sense → evaluate health → act" order a
real cyclic executive follows:

1. `adcs.updateEstimator(in, dt)` — EKF predict+correct; publishes
   `attitudeUncertaintyDeg` and (implicitly, via `gyroBiasEstimate`) the
   bias-corrected rate as `ADCS`'s own telemetry.
2. `fdir.evaluate(fdirIn, dt)` — reads that telemetry plus wheel health/
   battery SOC/commanded mode, and resolves `adcs.effectiveMode`. Runs
   even under `adcs.manualOverride` — a fault is still worth detecting/
   logging while a human has the stick; `adcs.control()` below is what
   actually respects `manualOverride`.
3. `adcs.control(in, dt)` — guidance + attitude control + desaturation +
   actuator allocation, executing whatever `effectiveMode` step 2 just
   resolved.

Deliberately no HAL/interface layer sits between `FlightSoftware` and the
harness — `step()` is a pure function of `(internal state, FSWInputs, dt)
-> FSWOutputs`, same shape as `ADCS::step()` itself (see "Hardware-
Abstraction Boundary" above). The harness's main loop is what bridges
between this and simulated hardware, via `Cubesat::sampleSensors()`/
`applyActuatorCommands()` (`src/core/Cubesat.h`/`.cpp`) — see "Orbital
Mechanics"'s note on the unified fixed-rate loop for exactly where those
calls happen relative to `world.step()`/`orbitPropagator.step()`.

`ADCS` also exposes a thin `step(in, dt)` convenience wrapper
(`updateEstimator()` + `effectiveMode = mode` + `control()`) for
FDIR-agnostic callers — this project's own `tests/test_adcs_control.cpp`
and `tests/test_flight_software.cpp` use it directly, since they exercise
guidance/control/estimation behavior without needing a fault monitor in
the loop.

### `SystemMode`

`SystemMode` (`SAFE`/`NOMINAL`, from `FlightSoftware::systemMode()`) is a
coarse, top-level "what is the spacecraft doing right now" concept —
coarser than `ADCS`'s own `PointingMode`, matching real flight software's
own layered mode structure. It's currently derived 1:1 from
`fdir.state()` (`FDIR` is this project's only source of "should we still
be doing the mission" today), but is deliberately exposed under this
stable, firmware-style name rather than requiring every caller to know
`FDIR`'s own state enum — so a future addition to what can force SAFE
(e.g. a thermal fault, once thermal is modeled) wouldn't change what
callers have to know about.

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
`solarFluxWm2 = 1361` (solar constant at 1 AU). Generation is additionally
zeroed while `EclipseModel::inEclipse()` (see "Orbital Mechanics" below)
says the real orbital position is in Earth's shadow — cylindrical shadow
only, not the true conical umbra/penumbra (a documented simplification,
not a missing model).

**Eclipse gating is applied by the harness, not the engine**: neither
`SolarPanel` nor `SunSensor` (in `spacecraft-dynamics-sim`) has any notion
of eclipse — both compute purely from the geometric sun direction they're
given, and both headers explicitly document that a scenario wanting
eclipse-aware behavior has to gate it externally. `main.cpp`
does this in two places: solar generation is zeroed (above), and
`SunSensor::Reading.valid` is additionally ANDed with `!inEclipse` before
being handed to `FSWInputs.sunSensor` — a real coarse sun sensor reports
no lock when the sun is physically blocked by Earth, not just when the
geometric direction is undefined. This matters beyond EPS: TRIAD fallback
(`computeTriadFallback`, `ADCS.cpp`) requires `in.sunSensor.valid`, so
without this gate it would happily solve a TRIAD attitude correction from
a sun reference the satellite couldn't actually observe during eclipse.
`StarTracker` is deliberately _not_ gated by eclipse — its own blinding
model (sun-exclusion angle, slew rate) is independent of it, and a real
star tracker generally sees better in eclipse (no sun/albedo glare), not
worse.

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

**Consumption** (harness-computed each FSW cycle, `main.cpp`):
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
computed with the _previous_ cycle's SOC as `FSWInputs.power.batterySoc`
(the same "read before this cycle's effects" relationship
`wheelTelemetry[i].speedRadS` already has with the wheel commands about to
be issued).
