#include <vgl/vgl.h>
#include <rigidbody/PhysicsWorld.h>
#include <rigidbody/actuators/ReactionWheel.h>
#include <rigidbody/actuators/Magnetorquer.h>
#include <rigidbody/sensors/IMU.h>
#include <rigidbody/sensors/Magnetometer.h>
#include <rigidbody/sensors/StarTracker.h>
#include <rigidbody/sensors/SunSensor.h>
#include <rigidbody/environment/central_body/CentralBodyMagneticField.h>
#include <rigidbody/orbit/OrbitState.h>
#include <rigidbody/orbit/OrbitalElements.h>
#include <rigidbody/orbit/OrbitForceModel.h>
#include <rigidbody/orbit/OrbitPropagator.h>
#include <rigidbody/orbit/OrbitFrames.h>
#include <rigidbody/orbit/OrbitTime.h>
#include <rigidbody/orbit/SunModel.h>
#include <rigidbody/orbit/EclipseModel.h>
#include <rigidbody/power/SolarPanel.h>
#include <rigidbody/power/Battery.h>
#include "ADCS.h"
#include "FlightTypes.h"
#include "ImGuiLayer.h"
#include "Telemetry.h"
#include <random>
#include <memory>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <string>
#include <vector>
#include <glm/gtc/constants.hpp>

// ---------------------------------------------------------------------------
// Full cubesat ADCS: pointing modes (Nadir/Sun/Detumble/Target/Slew/Fine/Reflect),
// an IMU and magnetometer as the only sensors the flight software ever
// reads (see ADCS.h/.cpp), a 4-wheel reaction wheel pyramid plus a 3-axis
// magnetorquer cluster as actuators, and randomly-occurring reaction wheel
// faults (reduced torque or complete failure). The magnetorquers can either
// sit idle (reaction-wheel-only detumble/pointing, the original behavior)
// or take over DETUMBLE via the classic B-dot law -- see the Controller
// panel's Detumble Actuator selector.
//
// Controls:
//   [1-7]   pointing mode: Nadir / Sun / Detumble / Target / Slew / Fine / Reflect
//   [Space] new random target (for Target/Slew/Fine/Reflect modes)
//   [T]     kick the body into a random tumble (to test Detumble)
//   [F]     force a wheel fault immediately (faults also occur on their own)
// ---------------------------------------------------------------------------
struct Cubesat
{
  RigidBody *body = nullptr;
  std::vector<ReactionWheel *> wheels;
  std::vector<Magnetorquer *> magnetorquers;
  IMU imu; // re-mounted off-center in buildCubesatPyramid()
  Magnetometer magnetometer;
  StarTracker starTracker; // boresight set in buildCubesatPyramid()
  SunSensor sunSensor;
  std::vector<SolarPanel> solarPanels; // one per body face -- see buildCubesatPyramid()
  Battery battery;
};

// Builds the simulated hardware AND the matching HardwareConfig ADCS::
// configure() needs, from the exact same geometry in one place -- keeping
// them in two separate functions risks the flight-software config quietly
// drifting out of sync with what the simulated actuators actually are.
static Cubesat buildCubesatPyramid(PhysicsWorld &world, HardwareConfig &outHw)
{
  Cubesat sat;
  sat.body = world.createBody(
      RigidBodyShape::BOX,
      glm::vec3(0.1f, 0.1f, 0.1f), // 10 x 10 x 10 cm
      1.33f);                      // max mass of 1U cubesat (kg)

  // Real orbital position (overwritten every frame from orbitState, in
  // ECI meters -- see main()) means this body's Z genuinely crosses zero
  // twice per orbit around an Earth centered at the world origin. The
  // engine's ground-collision resolution assumes a literal Z=0 ground
  // plane, which doesn't exist here, so this body opts out of it.
  sat.body->groundCollisionEnabled = false;

  // IMU board mounted in a corner of the bus, not at the center of mass --
  // like a real PCB, so its accelerometer isn't trivially always-zero (it
  // picks up centripetal/tangential terms from body rotation).
  sat.imu = IMU(glm::vec3(0.03f, 0.03f, 0.02f));

  // Pyramid layout: 4 wheels, each spin axis tilted `skew` from body +Z,
  // spaced 90 degrees apart in azimuth. Mounted in a small cluster near the
  // +Z face rather than at the body center, matching how a real RWA pyramid
  // bracket is bolted to one panel.
  const float skew = glm::radians(45.0f);
  const float mountRadius = 0.03f;
  const float mountHeight = 0.04f;

  for (int i = 0; i < 4; ++i)
  {
    float azimuth = glm::radians(45.0f) + i * glm::half_pi<float>(); // 45, 135, 225, 315 deg

    glm::vec3 axis(std::sin(skew) * std::cos(azimuth),
                   std::sin(skew) * std::sin(azimuth),
                   std::cos(skew));

    glm::vec3 mountPos(mountRadius * std::cos(azimuth),
                       mountRadius * std::sin(azimuth),
                       mountHeight);

    const float maxTorqueNm = 0.001f;                                       // same as the 3-wheel cubesat
    const float maxSpeedRadS = 6000.0f * (2.0f * glm::pi<float>() / 60.0f); // 6000 RPM max
    const float wheelInertia = 1e-6f;                                       // kg*m^2

    auto wheel = std::make_unique<ReactionWheel>(mountPos, axis, maxTorqueNm, maxSpeedRadS, wheelInertia);

    sat.wheels.push_back(wheel.get());
    outHw.wheels[i] = {axis, maxTorqueNm, maxSpeedRadS, wheelInertia};
    sat.body->addForceGenerator(std::move(wheel));
  }

  // Magnetorquer cluster: 3 mutually orthogonal rods along the body axes,
  // the standard cubesat layout (unlike the wheels' skewed pyramid, there's
  // no benefit to tilting a torque rod -- it has no momentum to distribute
  // across axes, so straight body-axis alignment gives the cleanest
  // allocation). 0.2 A*m^2 is a representative max moment for a 1U-class
  // torque rod.
  const glm::vec3 torquerAxes[3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  const glm::vec3 torquerMounts[3] = {{0.045f, 0, 0}, {0, 0.045f, 0}, {0, 0, -0.045f}};
  const float maxMomentAm2 = 0.2f;
  for (int i = 0; i < 3; ++i)
  {
    auto rod = std::make_unique<Magnetorquer>(torquerMounts[i], torquerAxes[i], maxMomentAm2);
    sat.magnetorquers.push_back(rod.get());
    outHw.torquers[i] = {torquerAxes[i], maxMomentAm2};
    sat.body->addForceGenerator(std::move(rod));
  }

  // Magnetometer mounted off-center like the IMU, opposite corner -- real
  // ADCS boards keep the magnetometer away from the torque rods/wheels
  // where practical, since their fields would otherwise swamp the sensor.
  // This model doesn't simulate that interference, but the placement still
  // reflects real layout practice.
  sat.magnetometer = Magnetometer(glm::vec3(-0.03f, -0.03f, 0.02f));

  // Star tracker boresight along body -Z (StarTracker's own default) --
  // opposite the +Z payload/pointing axis every guidance mode here aims,
  // so it isn't staring straight at whatever TARGET/SUN_POINTING/NADIR is
  // currently pointing +Z toward. Real placement follows the same logic:
  // keep the tracker away from the sun-facing/payload side.

  // Solar panels: one body-mounted cell array per face, the standard
  // cubesat layout (vs. a single sun-tracking array) -- whichever face(s)
  // happen to be sunward generate, the rest don't, so generation is a
  // direct function of attitude rather than something a gimbal hides.
  // Each face gets the full 10x10cm side as its cell area; ~28% is a
  // representative conversion efficiency for a triple-junction cubesat
  // cell (vs. ~20% for cheaper silicon).
  const float panelAreaM2 = sat.body->size.x * sat.body->size.y; // one 10cm x 10cm face
  const float panelEfficiency = 0.28f;
  const glm::vec3 panelNormals[6] = {
      {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
  for (const glm::vec3 &n : panelNormals)
    sat.solarPanels.emplace_back(n, panelAreaM2, panelEfficiency);

  // Battery: a representative small Li-ion pack for a 1U-class cubesat --
  // enough capacity that nominal operation trends toward full charge (this
  // sim has no orbital eclipse model, so there's no shadow period to drain
  // it against), while still being small enough that a deliberately harsh
  // test (Simulation tab's "Drain Battery" button, or a long run with poor
  // sun-facing geometry) can bring it down to FDIR's low-battery threshold.
  sat.battery = Battery(10.0f /* Wh */, 6.0f /* V empty */, 8.4f /* V full */, 0.8f /* initial SOC */);

  outHw.busInertiaTensor = sat.body->inertiaTensor;

  return sat;
}

// ---------------------------------------------------------------------------
// Random point on Earth's real surface, for an arbitrary ground-pointing
// target (TARGET/SLEW/FINE_POINTING modes). Placed at real Earth radius,
// not an arbitrary unit-sphere point -- now that spacecraftPositionWorld
// is a real ECI position (~6.9e6 m for LEO), a target near the origin
// would both be physically meaningless (inside Earth) and numerically
// broken: `target - spacecraftPositionWorld` with a ~1-unit target and a
// ~6.9e6-unit spacecraft position loses essentially all of target's
// contribution to float32 rounding (catastrophic cancellation) -- the
// direction would come out as just "toward Earth's center" regardless of
// the random target chosen. A target at comparable (Earth-radius) scale
// keeps this subtraction well-conditioned.
// ---------------------------------------------------------------------------
static glm::vec3 randomTarget()
{
  static std::mt19937 rng(std::random_device{}());
  static std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
  glm::vec3 v(dist(rng), dist(rng), dist(rng));
  return glm::normalize(v) * static_cast<float>(OrbitFrames::EARTH_RADIUS_M);
}

// ---------------------------------------------------------------------------
// Config
// ---------------------------------------------------------------------------
namespace Config
{
  // Satellite is a real 0.1m cubesat at a real ~6.9e6m orbital radius --
  // rendered at its actual size, it would be a sub-pixel dot from any
  // camera distance that also shows Earth. SATELLITE_VISUAL_SCALE
  // inflates every *drawn* dimension of the satellite (wireframe/wheels/
  // rods/arrows/mirror/grid/field arrows) around its real position, same
  // "not to scale but at the right place" idea already used for the sun
  // marker/B-field arrows -- it never touches the physical quantities
  // (RigidBody size, wheel inertia, mount offsets used for actual
  // dynamics) any of the FSW/physics math is computed from, only what
  // gets handed to the renderer. Applied as: draw the true local geometry
  // scaled up around the satellite's real center, i.e.
  // `satPos + (trueOffset) * SATELLITE_VISUAL_SCALE` for anything offset
  // from center, and `trueSize * SATELLITE_VISUAL_SCALE` for anything
  // sized -- see drawSatelliteWireframe/drawReactionWheels/etc.
  constexpr float SATELLITE_VISUAL_SCALE = 1000000.0f;

  // Camera: follows the satellite's real ECI position (see main()'s
  // orbit.setTarget() call each frame), so distances are tuned for a
  // scene that spans everything from the now-visually-inflated satellite
  // up to a ~6.9e6m orbital radius in the same world.
  //
  // MIN/INITIAL_DISTANCE are derived from SATELLITE_VISUAL_SCALE rather
  // than independent constants, so they can't drift out of sync with it
  // again if that scale ever changes: the satellite's own drawn geometry
  // reaches SATELLITE_VISUAL_SCALE * 0.25 from its center (the body-axis
  // arrows, drawSatelliteWireframe's largest extent), so MIN_DISTANCE
  // (0.5x the full scale, 2x the arrows' own reach) keeps the camera from
  // ending up inside the satellite's inflated geometry when fully zoomed
  // in.
  //
  // MAX_DISTANCE must stay well *inside* CAMERA_FAR: OrbitalCamera clamps
  // `distance` to [MIN_DISTANCE, MAX_DISTANCE], and if that range extends
  // past the far clip plane, zooming out toward MAX_DISTANCE pushes the
  // camera's target (and the whole scene around it) outside the far
  // plane's render range, which just makes everything vanish -- not what
  // "zoom out" should do. Earth's orbit diameter is ~1.37e7m; 1e8
  // (~100,000 km, ~7x that) comfortably frames the whole Earth+orbit
  // system with room to spare, and CAMERA_FAR sits a further 5x beyond
  // that so the camera is never anywhere near clipping its own target.
  // The near:far ratio this creates (~5e7) needs the logarithmic depth
  // buffer enabled in main() (gui.setLogDepth) to avoid z-fighting a
  // standard depth buffer can't handle at this range.
  constexpr float CAMERA_NEAR = 10.0f;
  constexpr float CAMERA_FAR = 5.0e8f;
  constexpr float CAMERA_FOV = 45.0f;
  constexpr float CAMERA_INITIAL_DISTANCE = SATELLITE_VISUAL_SCALE * 0.8f;
  constexpr float CAMERA_MIN_DISTANCE = SATELLITE_VISUAL_SCALE * 0.5f;
  constexpr float CAMERA_MAX_DISTANCE = 1.0e8f;
  constexpr float ZOOM_SENSITIVITY = 1.0f;
  constexpr float PAN_SENSITIVITY = 0.2f;

  // Coordinate grid: three walls of a box (floor + two back walls) meeting
  // at a corner behind/below the satellite, acting as a visual coordinate
  // reference rather than an infinite ground plane. Half-size/step here
  // are the *pre-SATELLITE_VISUAL_SCALE* values (still proportioned
  // against the real 10cm cubesat/0.25m body-axis arrows); the call site
  // in main() applies SATELLITE_VISUAL_SCALE, same as every other drawn
  // satellite dimension.
  constexpr float GRID_HALF_SIZE = 3.0f;
  constexpr float GRID_STEP = 0.5f;

  // A visible marker radius for world-frame point markers (the TARGET
  // pointing mode's ground target) that, like the satellite, would
  // otherwise render as a sub-pixel dot at real orbital-distance camera
  // ranges -- unrelated to SATELLITE_VISUAL_SCALE (this is a mark on
  // Earth's real surface, not part of the satellite), sized to be
  // visible (a large-city-scale dot) against Earth's real 6.371e6m
  // radius rather than to represent any real object's true size.
  constexpr float TARGET_MARKER_RADIUS_M = 5.0e4f;

  // Ground footprint / ground-track minimum elevation: the horizon-limited
  // case (0 deg) shows the full circle a satellite can geometrically see
  // any part of, not a specific ground station's usable pass window (which
  // would want a higher minimum, e.g. 10 deg, to exclude low-elevation
  // passes with poor link geometry -- not modeled here since there's no
  // ground-station concept yet).
  constexpr float FOOTPRINT_MIN_ELEVATION_DEG = 0.0f;

  // Ground track: how much history to keep and how often to sample it.
  // Sampling once every GROUND_TRACK_SAMPLE_INTERVAL_S of mission time
  // (not every frame) at GROUND_TRACK_MAX_POINTS keeps the trail spanning
  // multiple orbits without growing unbounded over a long-running mission.
  constexpr int GROUND_TRACK_MAX_POINTS = 400;
  constexpr float GROUND_TRACK_SAMPLE_INTERVAL_S = 15.0f;

  // Plot panel (world space, Y=-1.5 plane, X in [-0.5, 0.5], Z up)
  const glm::vec3 PLOT_ORIGIN{-0.5f, -1.5f, 0.0f};
  constexpr float PLOT_WIDTH = 1.0f;
  constexpr float PLOT_HEIGHT = 0.18f;
  constexpr float PLOT_GAP = 0.05f;

  constexpr float TUMBLE_KICK_RAD_S = 1.5f;

  constexpr int TELEMETRY_HISTORY_SAMPLES = 300;

  // The sun's real angular diameter as seen from Earth/LEO, ~32 arcminutes
  // -- the sun marker's radius is sized every frame from this and its
  // current (arbitrary, scaled-for-visibility) distance from the
  // satellite, rather than being a fixed prop radius, so it actually
  // reads as "how big the sun really looks," not just a bright dot.
  constexpr float SUN_ANGULAR_DIAMETER_DEG = 32.0f / 60.0f;

  // A flat mirror mounted on the +Z face (same axis every pointing mode
  // aims), just for visualizing the sun-reflection geometry -- not wired
  // into ADCS/guidance at all.
  const glm::vec3 MIRROR_SIZE{0.09f, 0.09f, 0.003f}; // thin along its own normal (local Z)
  const glm::vec3 MIRROR_NORMAL_BODY{0.0f, 0.0f, 1.0f};
  constexpr float REFLECTED_RAY_LENGTH = 1.0f;

  // Orbit visualization: PhysicsWorld's own coordinate frame *is* ECI
  // here -- Earth's center is the world origin (real meters), and
  // sat.body->position is bridged from orbitState every frame (see
  // main()), so there's no separate render scale/offset to track
  // anymore: Earth is drawn at its real radius, at the real origin, and
  // the satellite is wherever its real orbital position puts it.
  constexpr int ORBIT_PATH_POINTS = 120;       // segments in the predicted-path polyline
  constexpr float ORBIT_PATH_REFRESH_S = 5.0f; // real seconds between path recomputes

  // EPS (electrical power subsystem) power budget: representative
  // wattages for every load on the bus, summed each ADCS cycle against
  // what the solar panels generate that same cycle -- see the FLIGHT
  // SOFTWARE block in main(). Generation is gated by EclipseModel::
  // inEclipse() against the real orbital position (see rigidbody/orbit/
  // EclipseModel.h) -- cylindrical shadow only, not the true conical
  // umbra/penumbra, a documented simplification (see docs/ALGORITHMS.md).
  constexpr float SOLAR_FLUX_WM2 = 1361.0f; // solar constant at 1 AU

  constexpr float POWER_OBC_BASELINE_W = 0.4f; // onboard computer + FSW housekeeping, always on
  constexpr float POWER_IMU_W = 0.05f;
  constexpr float POWER_MAGNETOMETER_W = 0.03f;
  constexpr float POWER_STAR_TRACKER_W = 0.35f; // star trackers are the power-hungriest sensor on a cubesat bus
  constexpr float POWER_SUN_SENSOR_W = 0.01f;

  // Reaction wheel: a small standby/electronics draw per wheel regardless
  // of command, plus mechanical power (torque * speed) over an assumed
  // motor efficiency -- the same "idle + effort-proportional" shape a real
  // motor draws, not a flat number that ignores what the wheel is actually
  // being asked to do.
  constexpr float WHEEL_IDLE_POWER_W = 0.02f;
  constexpr float WHEEL_MOTOR_EFFICIENCY = 0.6f;

  // Magnetorquer: idle electronics draw plus a resistive coil term
  // proportional to the commanded dipole moment (a real rod's power is
  // I^2*R, i.e. proportional to moment^2, but a linear approximation is
  // close enough at these scales and avoids implying more precision about
  // the coil's actual resistance than this project has any basis for).
  constexpr float TORQUER_IDLE_POWER_W = 0.01f;
  constexpr float TORQUER_POWER_PER_AM2_W = 0.5f;
}

// ---------------------------------------------------------------------------
// Rolling history of each sensor's reading magnitude, for the Sensors &
// Actuators panel's plots -- one channel per sensor (not per axis) to keep
// the panel compact; per-axis values are still shown as text alongside each
// plot. Pushed once per ADCS cycle (20 Hz, matching when a new reading
// actually exists), not once per render frame.
struct SensorTelemetry
{
  TelemetryChannel gyroMagDegS;
  TelemetryChannel accelMagMs2;
  TelemetryChannel magFieldMagUt;
  TelemetryChannel estimatedPointingErrorDeg; // what the FSW itself computes/would act on
  TelemetryChannel truePointingErrorDeg;      // ground truth, diagnostic only
  TelemetryChannel batterySocPct;
  TelemetryChannel netPowerW; // generation minus consumption -- positive charges, negative discharges

  explicit SensorTelemetry(int samples)
      : gyroMagDegS(samples), accelMagMs2(samples), magFieldMagUt(samples),
        estimatedPointingErrorDeg(samples), truePointingErrorDeg(samples),
        batterySocPct(samples), netPowerW(samples) {}
};

// ---------------------------------------------------------------------------
// Draw helpers
// ---------------------------------------------------------------------------

// Body drawn as a 12-edge wireframe box instead of a solid box.
void drawSatelliteWireframe(GUI &gui, RigidBody *sat)
{
  glm::vec3 h = sat->size * 0.5f * Config::SATELLITE_VISUAL_SCALE;
  glm::quat q = sat->orientation;
  glm::vec3 p = sat->position;
  const glm::vec3 color{1.0f, 1.0f, 0.0f};

  glm::vec3 c[8] = {
      p + q * glm::vec3(-h.x, -h.y, -h.z),
      p + q * glm::vec3(+h.x, -h.y, -h.z),
      p + q * glm::vec3(+h.x, +h.y, -h.z),
      p + q * glm::vec3(-h.x, +h.y, -h.z),
      p + q * glm::vec3(-h.x, -h.y, +h.z),
      p + q * glm::vec3(+h.x, -h.y, +h.z),
      p + q * glm::vec3(+h.x, +h.y, +h.z),
      p + q * glm::vec3(-h.x, +h.y, +h.z),
  };

  // Bottom face (4)
  gui.drawLine(c[0], c[1], color);
  gui.drawLine(c[1], c[2], color);
  gui.drawLine(c[2], c[3], color);
  gui.drawLine(c[3], c[0], color);
  // Top face (4)
  gui.drawLine(c[4], c[5], color);
  gui.drawLine(c[5], c[6], color);
  gui.drawLine(c[6], c[7], color);
  gui.drawLine(c[7], c[4], color);
  // Vertical edges (4)
  gui.drawLine(c[0], c[4], color);
  gui.drawLine(c[1], c[5], color);
  gui.drawLine(c[2], c[6], color);
  gui.drawLine(c[3], c[7], color);

  float arrowLength = 0.25f * Config::SATELLITE_VISUAL_SCALE;
  gui.drawArrow(p, p + q * glm::vec3(1, 0, 0) * arrowLength, glm::vec3(1, 0, 0));
  gui.drawArrow(p, p + q * glm::vec3(0, 1, 0) * arrowLength, glm::vec3(0, 1, 0));
  gui.drawArrow(p, p + q * glm::vec3(0, 0, 1) * arrowLength, glm::vec3(0, 0, 1));
}

// Wheels drawn as flat cylinders at their actual mount position/orientation,
// with a speed arrow from each wheel's center. Color communicates health
// first (magenta = degraded, near-black = dead) and saturation only for
// wheels that are actually healthy.
void drawReactionWheels(GUI &gui, const std::vector<ReactionWheel *> &reactionWheels, RigidBody *sat)
{
  const float wheelRadius = 0.02f * Config::SATELLITE_VISUAL_SCALE;
  const float wheelThickness = 0.006f * Config::SATELLITE_VISUAL_SCALE;
  const float arrowLength = 0.05f * Config::SATELLITE_VISUAL_SCALE;

  glm::vec3 totalAngular{0};
  for (auto &wheel : reactionWheels)
  {
    // wheel->getWorldMountPosition(*sat) computes sat->position +
    // sat->orientation*mountPositionBody *inside the engine*, in float32,
    // before we ever see the result -- at a real orbital position
    // (~6.9e6 m), float32's ULP there (~0.8 m) is *larger* than the
    // ~0.03 m mount offset itself, so that addition rounds the offset
    // away almost entirely before SATELLITE_VISUAL_SCALE ever gets
    // applied to it (this was the wheels'/rods' actual rendering bug --
    // not a scale-factor mistake, a precision loss that happened before
    // any of this function's own math runs). Fixed by reading
    // mountPositionBody directly (a public field) and applying
    // SATELLITE_VISUAL_SCALE to the *offset* first -- turning a ~0.03 m
    // value into a ~600 m one, comfortably above that same ULP -- before
    // rotating and adding it to sat->position, exactly the order
    // drawMirror already uses for its own mount offset.
    glm::vec3 worldPos = sat->position + sat->orientation * (wheel->mountPositionBody * Config::SATELLITE_VISUAL_SCALE);
    glm::vec3 worldAxis = wheel->getWorldSpinAxis(*sat);

    glm::vec3 color;
    if (wheel->healthFactor <= 0.01f)
      color = {0.15f, 0.15f, 0.15f}; // dead
    else if (wheel->healthFactor < 0.99f)
      color = {0.85f, 0.1f, 0.85f}; // degraded
    else
    {
      float satRatio = wheel->getSaturationRatio();
      float absSatRatio = std::abs(satRatio);
      if (absSatRatio < 0.5f)
        color = {0, 1, 0};
      else if (absSatRatio < 0.9f)
        color = {1, 1, 0};
      else
        color = {1, 0, 0};
    }

    // Flat "puck": thin along the spin axis, wide across it.
    gui.drawCylinder(worldPos, wheelRadius, wheelThickness, worldAxis, glm::quat(1, 0, 0, 0), color);

    // Speed arrow from the wheel's own center, along its spin axis --
    // reflects actual current speed regardless of health.
    float satRatio = wheel->getSaturationRatio();
    gui.drawArrow(worldPos, worldPos + worldAxis * arrowLength * satRatio, color);
    totalAngular += worldAxis * satRatio;
  }
  gui.drawArrow(sat->position, sat->position + totalAngular * arrowLength * 4.0f, {1.0f, 0.65f, 0});
}

// Torque rods drawn as thin cylinders along their mounted axis (unlike the
// wheels' flat pucks -- a physical torque rod is a long, thin coil, not a
// disc). Color scales with saturation (how close to max commanded moment),
// same 3-stop green/yellow/red convention drawReactionWheels uses; an
// arrow from the rod's center along its axis shows the sign/magnitude of
// the currently commanded dipole moment.
static void drawMagnetorquers(GUI &gui, const std::vector<Magnetorquer *> &magnetorquers, RigidBody *sat)
{
  const float rodRadius = 0.006f * Config::SATELLITE_VISUAL_SCALE;
  const float rodLength = 0.07f * Config::SATELLITE_VISUAL_SCALE;
  const float arrowLength = 0.06f * Config::SATELLITE_VISUAL_SCALE;

  for (auto &rod : magnetorquers)
  {
    // See drawReactionWheels' equivalent comment -- same precision fix:
    // scale the local mount offset before rotating/adding it, not after.
    glm::vec3 worldPos = sat->position + sat->orientation * (rod->mountPositionBody * Config::SATELLITE_VISUAL_SCALE);
    glm::vec3 worldAxis = rod->getWorldAxis(*sat);

    float satRatio = rod->getSaturationRatio();
    float absSatRatio = std::abs(satRatio);
    glm::vec3 color;
    if (absSatRatio < 0.5f)
      color = {0.2f, 0.6f, 1.0f}; // blue-ish (idle/light use) to distinguish from wheels' green
    else if (absSatRatio < 0.9f)
      color = {1, 1, 0};
    else
      color = {1, 0, 0};

    gui.drawCylinder(worldPos, rodRadius, rodLength, worldAxis, glm::quat(1, 0, 0, 0), color);
    gui.drawArrow(worldPos, worldPos + worldAxis * arrowLength * satRatio, color);
  }
}

// Star tracker boresight: a thin line out from the body along its current
// pointing direction, colored by the same valid/blinded/no-correction
// status the Sensors tab shows -- lets you visually connect "the tracker
// just went red" with the boresight swinging toward the sun marker.
static void drawStarTracker(GUI &gui, const StarTracker &tracker, RigidBody *sat, ADCS &adcs)
{
  const float length = 0.5f * Config::SATELLITE_VISUAL_SCALE;
  glm::vec3 worldAxis = sat->orientation * tracker.boresightBody;

  glm::vec3 color;
  if (adcs.starTrackerValid)
    color = {0.3f, 1.0f, 0.4f};
  else if (adcs.triadFallbackUsed)
    color = {1.0f, 0.8f, 0.2f};
  else
    color = {1.0f, 0.4f, 0.2f};

  gui.drawArrow(sat->position, sat->position + worldAxis * length, color);
}

// A flat mirror bolted to the +Z face, purely for visualizing sun-
// reflection geometry -- see Config::MIRROR_* and drawSunReflection().
// Not a physics body and not wired into ADCS/guidance at all.
static void drawMirror(GUI &gui, RigidBody *sat)
{
  // mountOffsetBody is computed from the satellite's true (0.1m-scale)
  // size, then scaled -- this is a pure body-frame local offset, not yet
  // rotated into world space, so scaling it here (before applying
  // orientation) is equivalent to and simpler than the
  // offset-from-center trick drawReactionWheels/drawMagnetorquers use.
  glm::vec3 mountOffsetBody = Config::MIRROR_NORMAL_BODY * (sat->size.z * 0.5f + Config::MIRROR_SIZE.z * 0.5f) * Config::SATELLITE_VISUAL_SCALE;
  glm::vec3 worldPos = sat->position + sat->orientation * mountOffsetBody;
  gui.drawBox(worldPos, Config::MIRROR_SIZE * Config::SATELLITE_VISUAL_SCALE, sat->orientation, {0.85f, 0.92f, 0.98f});
}

// Draws the incoming ray from the sun to the mirror, and the outgoing
// (reflected) ray away from it, via the ordinary law of reflection
// (angle of incidence = angle of reflection about the mirror's normal).
// The reflected ray is only drawn when the mirror's front face is
// actually sun-facing -- reflecting a ray that's hitting the mirror's
// back would be nonsense, not just an unlikely case a real mirror can't
// do either.
static void drawSunReflection(GUI &gui, RigidBody *sat, const glm::vec3 &sunPosition)
{
  // Same scaled mountOffsetBody as drawMirror -- must match exactly so
  // the reflection geometry originates from the same point the mirror is
  // actually drawn at.
  glm::vec3 mountOffsetBody = Config::MIRROR_NORMAL_BODY * (sat->size.z * 0.5f + Config::MIRROR_SIZE.z * 0.5f) * Config::SATELLITE_VISUAL_SCALE;
  glm::vec3 mirrorPos = sat->position + sat->orientation * mountOffsetBody;
  glm::vec3 normalWorld = glm::normalize(sat->orientation * Config::MIRROR_NORMAL_BODY);

  // sunPosition is the real Sun position (~1.5e11 m away, see main()) --
  // mirrorPos sitting a few km from the satellite's real orbital position
  // is negligible against that distance, so this direction is unaffected
  // by the mirror-offset visual scaling above.
  glm::vec3 incidentDir = glm::normalize(mirrorPos - sunPosition); // sun -> mirror
  gui.drawLine(sunPosition, mirrorPos, {1.0f, 0.5f, 0.9f});

  // Front face is illuminated only if the normal points back toward the
  // sun relative to the incoming ray, i.e. dot(normal, -incident) > 0.
  if (glm::dot(normalWorld, -incidentDir) <= 0.0f)
    return;

  glm::vec3 reflectedDir = incidentDir - 2.0f * glm::dot(incidentDir, normalWorld) * normalWorld;
  gui.drawLine(mirrorPos, mirrorPos + reflectedDir * Config::REFLECTED_RAY_LENGTH * Config::SATELLITE_VISUAL_SCALE, {1.0f, 0.5f, 0.9f});
}

// Visualizes the ambient field the satellite is sitting in: a small grid of
// arrows around the body, all pointing the same direction/magnitude, since
// the field varies negligibly over a cubesat-sized volume (its gradient
// scale is Earth-sized, not cubesat-sized) -- what varies is the field
// *over time* as the real orbit (see OrbitPropagator/CentralBodyMagneticField
// in main()) sweeps through it. A distinct, brighter arrow at the body
// itself marks the same vector so it reads clearly against the grid.
static void drawMagneticField(GUI &gui, const glm::vec3 &fieldWorldT, glm::vec3 satPos)
{
  // Scales a ~20-60 uT LEO field into a visible arrow length, then applies
  // SATELLITE_VISUAL_SCALE like every other satellite-local dimension so
  // the field arrows stay proportionate to the now-much-bigger satellite
  // instead of shrinking to nothing next to it.
  constexpr float FIELD_VISUAL_SCALE = 8000.0f;
  const glm::vec3 fieldColor{0.2f, 0.9f, 0.9f};

  glm::vec3 arrow = fieldWorldT * FIELD_VISUAL_SCALE * Config::SATELLITE_VISUAL_SCALE;

  gui.drawArrow(satPos, satPos + arrow, fieldColor, 2.0f);

  const int gridN = 2; // -gridN..+gridN in each of x,y -> 5x5 grid
  const float spacing = 0.6f * Config::SATELLITE_VISUAL_SCALE;
  const float gridHeight = -0.7f * Config::SATELLITE_VISUAL_SCALE; // below the satellite, out of the way of the sat/wheels/rods
  for (int ix = -gridN; ix <= gridN; ++ix)
  {
    for (int iy = -gridN; iy <= gridN; ++iy)
    {
      glm::vec3 base = satPos + glm::vec3(ix * spacing, iy * spacing, gridHeight);
      gui.drawArrow(base, base + arrow, fieldColor * 0.6f);
    }
  }
}

// Predicted orbit path: propagates a *copy* of the current orbital state
// forward for one estimated period (vis-viva, from the current
// position/velocity -- exact for the unperturbed two-body case, a good
// approximation with J2 included since J2 doesn't secularly change
// semi-major axis to first order) using the same two-body+J2 force model
// the real propagator uses, sampling numPoints evenly-spaced points.
// Returns real ECI-meter points (world origin = Earth's center, matching
// PhysicsWorld's own frame here), ready to draw directly. Recomputed
// periodically (not every frame) by the caller -- see
// ORBIT_PATH_REFRESH_S -- since nothing here is cheap enough to be worth
// redoing 60 times a second for a path that only drifts slowly (mainly
// from J2) cycle to cycle.
static std::vector<glm::vec3> computePredictedOrbitPath(const OrbitState &current, int numPoints)
{
  double mu = TwoBodyGravity{}.mu;
  double rMag = glm::length(current.position);
  double vMagSq = glm::dot(current.velocity, current.velocity);
  double semiMajorAxisM = 1.0 / (2.0 / rMag - vMagSq / mu); // vis-viva
  double period = 2.0 * M_PI * std::sqrt(semiMajorAxisM * semiMajorAxisM * semiMajorAxisM / mu);

  OrbitPropagator pathPropagator;
  pathPropagator.addForceModel(std::make_unique<TwoBodyGravity>());
  pathPropagator.addForceModel(std::make_unique<J2Perturbation>());

  OrbitState state = current;
  double dt = period / numPoints;

  std::vector<glm::vec3> points;
  points.reserve(numPoints + 1);
  points.push_back(glm::vec3(state.position));
  for (int i = 0; i < numPoints; i++)
  {
    pathPropagator.step(state, dt);
    points.push_back(glm::vec3(state.position));
  }
  return points;
}

static void drawOrbitPath(GUI &gui, const std::vector<glm::vec3> &pathPoints)
{
  const glm::vec3 pathColor{0.3f, 0.7f, 1.0f};
  for (size_t i = 0; i + 1 < pathPoints.size(); i++)
    gui.drawLine(pathPoints[i], pathPoints[i + 1], pathColor, 1.5f);
}

// World coordinate-axes gizmo: three world-aligned (not body-aligned)
// arrows anchored to a fixed offset from the *camera*, not the scene, so
// they sit in a fixed screen corner and visibly rotate as the user orbits
// the camera -- the same idea as the axis gizmo in Blender/Unity/
// constellation-sim. VGL has no second-viewport/HUD render pass
// (GUI::beginFrame() sets the view/projection uniforms once per frame
// from `camera`, see GUI.cpp -- there's no mid-frame camera swap to draw
// a classic screen-space overlay with), so this gets the same visual
// result entirely in world space instead: placed a small *fixed* world
// distance in front of the camera (independent of the main camera's
// current zoom distance to the satellite/Earth, which ranges from
// hundreds of thousands to tens of millions of units), offset toward one
// corner using the camera's own basis vectors and FOV/aspect, sized
// proportionally to that same fixed distance. Because the scene the
// gizmo might otherwise be confused with (Earth, the orbit path, the
// satellite) is always vastly farther from the camera than this, it
// naturally renders in front via ordinary depth testing -- no separate
// pass or depth-clear needed.
// World coordinate-axes gizmo, drawn in **screen space** via ImGui's
// foreground draw list -- not as 3D world geometry. An earlier version of
// this drew three arrows anchored to a fixed world-space offset from the
// camera, sized to look constant regardless of zoom; that technique is
// fundamentally fighting the 3D pipeline (depth testing against a scene
// spanning 0.1m to 1e8m, the logarithmic depth buffer's own precision
// characteristics at close range, perspective distortion changing the
// arrows' apparent proportions with FOV/aspect) to fake something that
// isn't really a 3D object at all -- a fixed-screen-position orientation
// indicator. constellation-sim's SatelliteRenderer::drawAxesOverlay()
// does this the direct way: skip 3D rendering for the gizmo entirely,
// project each world axis (a unit vector) onto the screen using simple
// dot products against the camera's own right/up basis vectors (an
// orthographic-style projection -- exactly right here, since the gizmo
// is meant to show *orientation only*, not position or perspective), and
// draw the result as 2D lines/triangles directly. No depth testing, no
// clip planes, no scale/distance math to get right -- ported here with
// this project's own axis color convention.
static void drawWorldAxesGizmo(GUI &gui)
{
  const ImGuiIO &io = ImGui::GetIO();
  ImDrawList *dl = ImGui::GetForegroundDrawList();

  const float margin = 80.0f; // pixels from the corner
  const float arm = 50.0f;    // pixel length of each fully-foreshortened axis
  const ImVec2 origin(margin, io.DisplaySize.y - margin); // bottom-left corner

  dl->AddCircleFilled(origin, arm * 0.65f, IM_COL32(0, 0, 0, 120), 32);

  // camera right -> screen +X, camera up -> screen -Y (screen Y grows
  // downward) -- dot(axis, basis) is exactly the orthographic projection
  // of a unit axis onto that screen direction, foreshortened correctly
  // as the camera rotates since it's already just the cosine of the
  // angle between them.
  glm::vec3 camRight = gui.camera.getRight();
  glm::vec3 camUp = gui.camera.getUpVector();
  auto project = [&](glm::vec3 axisDir) -> ImVec2
  {
    float sx = glm::dot(camRight, axisDir) * arm;
    float sy = -glm::dot(camUp, axisDir) * arm;
    return ImVec2(origin.x + sx, origin.y + sy);
  };

  // Same X=red/Y=green/Z=blue convention already used for the satellite's
  // own body-axis arrows (drawSatelliteWireframe) and the coordinate grid
  // (drawGridPlane) -- this reads as "the same axis colors, just showing
  // world orientation instead of body orientation."
  struct Axis
  {
    glm::vec3 dir;
    ImU32 color;
  };
  const Axis axes[3] = {
      {{1.0f, 0.0f, 0.0f}, IM_COL32(217, 77, 82, 230)},
      {{0.0f, 1.0f, 0.0f}, IM_COL32(82, 173, 102, 230)},
      {{0.0f, 0.0f, 1.0f}, IM_COL32(82, 122, 217, 230)},
  };

  for (const Axis &axis : axes)
  {
    ImVec2 tip = project(axis.dir);
    dl->AddLine(origin, tip, axis.color, 2.0f);

    float dx = tip.x - origin.x, dy = tip.y - origin.y;
    float len = std::sqrt(dx * dx + dy * dy);
    if (len < 1.0f)
      continue; // axis nearly edge-on to the screen -- no visible arrowhead to draw
    float ux = dx / len, uy = dy / len;
    float px = -uy * 4.0f, py = ux * 4.0f;
    dl->AddTriangleFilled(
        ImVec2(tip.x, tip.y),
        ImVec2(tip.x - ux * 9.0f + px, tip.y - uy * 9.0f + py),
        ImVec2(tip.x - ux * 9.0f - px, tip.y - uy * 9.0f - py),
        axis.color);
  }
}

// Earth: a textured sphere at the world origin, at its real radius --
// PhysicsWorld's own frame here *is* ECI (see main()), so there's no
// separate render scale/offset.
//
// Two rotations, composed in order:
//  1. Pole alignment: VGL's sphere mesh (MeshGen::sphere) maps its UV v=0
//     (mesh ring index 0) to the mesh's local +Y -- but VGL's Texture
//     loader calls stbi_set_flip_vertically_on_load(true) (see
//     VGL/src/Texture.cpp), so the pixel row that ends up at v=0 is the
//     *bottom* of the source image, not the top. For a standard
//     equirectangular Earth texture (top row = North Pole), that means
//     mesh local +Y actually samples the South Pole, and local -Y
//     samples the North Pole -- the opposite of the naive assumption.
//     A -90 deg rotation about X maps local -Y (the real North Pole
//     content) to world +Z (this scene's "up"/polar convention, the same
//     axis GMST/OrbitFrames rotates about) -- independent of time, it
//     only fixes the mesh's pole axis, it isn't itself Earth's rotation.
//     (A previous version of this used +90 deg, reasoning from the mesh
//     UVs alone without accounting for the texture-loader flip -- that
//     put the correct polar axis in place but with North and South
//     swapped, which is exactly the "sideways, then upside-down" order
//     these two bugs actually showed up in.)
//  2. Spin: +Z is now the correct polar axis, so rotating about it by the
//     current Greenwich sidereal angle is the real ECEF->ECI rotation
//     (same convention orbitState/OrbitFrames.h use) -- this is what
//     actually makes Earth visibly rotate over time.
// Composed as spin * poleAlignment (apply pole alignment first, then
// spin about the now-correct axis -- quaternion composition applies the
// right-hand operand first).
static void drawEarth(GUI &gui, const Texture &earthTexture, double currentJd)
{
  glm::quat poleAlignment = glm::angleAxis(glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
  float gmst = static_cast<float>(OrbitFrames::gmstRad(currentJd));
  glm::quat spin = glm::angleAxis(gmst, glm::vec3(0.0f, 0.0f, 1.0f));
  glm::quat earthRotation = spin * poleAlignment;
  gui.drawTexturedSphere(glm::vec3(0.0f), static_cast<float>(OrbitFrames::EARTH_RADIUS_M), earthRotation, earthTexture);
}

// Coverage half-angle (radians): the angular radius, from Earth's center,
// of the surface circle a satellite at satPosEci can see at or above
// minElevationRad -- shared by the 3D footprint (drawGroundFootprint) and
// the 2D ground-track minimap's footprint overlay, so the two can never
// drift out of sync with each other.
//
// With eta the satellite's nadir angle at which the local horizon
// (elevation = minElevationRad) is reached,
//   sin(eta) = R * cos(eps) / r         (R = Earth radius, r = sat radius, eps = min elevation)
//   rho = pi/2 - eps - eta
// standard ground-coverage geometry (a right triangle formed by Earth's
// center, the satellite, and the point on the horizon circle). Returns
// <= 0 if the satellite is too low/close for any point to reach
// minElevationRad, in which case there's no footprint to draw.
static double computeFootprintHalfAngleRad(const glm::dvec3 &satPosEci, double minElevationRad)
{
  double r = glm::length(satPosEci);
  double R = OrbitFrames::EARTH_RADIUS_M;
  double eta = std::asin(glm::clamp(R * std::cos(minElevationRad) / r, 0.0, 1.0));
  return glm::half_pi<double>() - minElevationRad - eta;
}

// Ground footprint: the small circle on Earth's surface a satellite at
// satPosEci can see at or above minElevationRad, plus a dashed nadir line
// from the surface up to the satellite -- ported from constellation-sim's
// SatelliteRenderer::drawCoverageFootprint (same geometry, this project's
// own real ECI position/PhysicsWorld-frame convention instead of a
// separately-scaled scene).
static void drawGroundFootprint(GUI &gui, const glm::dvec3 &satPosEci, double minElevationRad)
{
  double rho = computeFootprintHalfAngleRad(satPosEci, minElevationRad);
  if (rho <= 0.0)
    return; // satellite too low/close for any point to reach minElevationRad

  float cosRho = static_cast<float>(std::cos(rho));
  float sinRho = static_cast<float>(std::sin(rho));

  glm::vec3 sub = glm::normalize(glm::vec3(satPosEci)); // sub-satellite direction

  // Orthonormal basis perpendicular to sub, for tracing the small circle.
  glm::vec3 ref = (std::abs(glm::dot(sub, glm::vec3(0, 0, 1))) > 0.99f) ? glm::vec3(1, 0, 0) : glm::vec3(0, 0, 1);
  glm::vec3 t1 = glm::normalize(glm::cross(sub, ref));
  glm::vec3 t2 = glm::cross(sub, t1); // already unit (sub, t1 orthonormal)

  const int segments = 96;
  const float surfaceOffset = static_cast<float>(OrbitFrames::EARTH_RADIUS_M) * 1.001f; // just above the surface, avoids z-fighting
  const glm::vec3 footprintColor{0.0f, 0.85f, 1.0f};

  glm::vec3 prev{0.0f};
  for (int i = 0; i <= segments; i++)
  {
    float angle = glm::two_pi<float>() * i / segments;
    glm::vec3 dir = sub * cosRho + (t1 * std::cos(angle) + t2 * std::sin(angle)) * sinRho;
    glm::vec3 pt = dir * surfaceOffset;
    if (i > 0)
      gui.drawLine(prev, pt, footprintColor, 1.8f);
    prev = pt;
  }

  // Dashed nadir line, surface straight up to the satellite.
  glm::vec3 surfacePt = sub * surfaceOffset;
  glm::vec3 satPt = glm::vec3(satPosEci);
  const int dashSegments = 20;
  for (int i = 0; i < dashSegments; i += 2) // skip every other -- dashed look
  {
    glm::vec3 a = glm::mix(surfacePt, satPt, static_cast<float>(i) / dashSegments);
    glm::vec3 b = glm::mix(surfacePt, satPt, static_cast<float>(i + 1) / dashSegments);
    gui.drawLine(a, b, footprintColor * 0.8f, 0.8f);
  }
}

// 2D ground-track minimap: the Earth texture as a flat equirectangular
// background image, the ground-track history as a fading trail, the
// footprint circle projected onto the same map, and a marker at the
// current sub-satellite point -- ported from constellation-sim's
// SatelliteRenderer's "GROUND TRACK" panel (drawMercatorWindow's logic,
// in their code, ended up inlined into the panel that calls it; same
// here).
//
// UV flip note: VGL's Texture loader flips images vertically on load
// (stbi_set_flip_vertically_on_load -- see drawEarth's comment on why
// that mattered for the 3D globe's pole alignment). ImGui::Image's UV0/UV1
// arguments compensate for the same flip here, the same way
// constellation-sim's own minimap does -- (0,1)/(1,0) instead of the
// usual (0,0)/(1,1), so the displayed 2D map still reads north-up.
static void drawGroundTrackMinimap(const Texture &earthTexture,
                                   const std::vector<glm::vec2> &groundTrack,
                                   const glm::dvec3 &satPosEci, double thetaGstRad,
                                   double minElevationRad)
{
  if (!ImGui::BeginChild("##ground_track", ImVec2(0.0f, 220.0f), false, ImGuiWindowFlags_NoScrollbar))
  {
    ImGui::EndChild();
    return;
  }

  ImVec2 cp = ImGui::GetCursorScreenPos();
  ImVec2 cs = ImGui::GetContentRegionAvail();

  ImGui::Image(static_cast<ImTextureID>(static_cast<uintptr_t>(earthTexture.id())),
              cs, ImVec2(0.0f, 1.0f), ImVec2(1.0f, 0.0f));

  ImDrawList *dl = ImGui::GetWindowDrawList();

  // Equirectangular projection: lon in [-180,180] -> x in [0, width],
  // lat in [-90,90] -> y in [0, height] (north at the top).
  auto toScreen = [&](float latDeg, float lonDeg) -> ImVec2
  {
    return ImVec2(cp.x + (lonDeg + 180.0f) / 360.0f * cs.x,
                 cp.y + (90.0f - latDeg) / 180.0f * cs.y);
  };

  // Ground-track trail, fading from dim to bright toward the current
  // position. Segments that cross the +-180 deg seam (a huge apparent
  // longitude jump) are skipped rather than drawn as a spurious line all
  // the way across the map.
  int trailCount = static_cast<int>(groundTrack.size());
  for (int i = 1; i < trailCount; i++)
  {
    float lon0 = groundTrack[i - 1].y, lon1 = groundTrack[i].y;
    if (std::abs(lon1 - lon0) > 90.0f)
      continue;
    float age = static_cast<float>(i) / trailCount;
    ImU32 col = IM_COL32(static_cast<int>(20 + 20 * age), static_cast<int>(120 + 135 * age), static_cast<int>(220 - 20 * age), 220);
    dl->AddLine(toScreen(groundTrack[i - 1].x, groundTrack[i - 1].y),
               toScreen(groundTrack[i].x, groundTrack[i].y), col, 1.5f);
  }

  // Footprint circle, projected onto the map -- traced the same way the 3D
  // version does (see drawGroundFootprint), each point converted to
  // geodetic individually since the small-circle-around-nadir shape isn't
  // preserved under an equirectangular projection (especially near the
  // poles).
  double rho = computeFootprintHalfAngleRad(satPosEci, minElevationRad);
  if (rho > 0.0)
  {
    glm::dvec3 sub = glm::normalize(satPosEci);
    glm::dvec3 ref = (std::abs(sub.z) > 0.99) ? glm::dvec3(1.0, 0.0, 0.0) : glm::dvec3(0.0, 0.0, 1.0);
    glm::dvec3 t1 = glm::normalize(glm::cross(sub, ref));
    glm::dvec3 t2 = glm::cross(sub, t1);
    double cosRho = std::cos(rho), sinRho = std::sin(rho);

    const int segments = 72;
    bool hasPrev = false;
    ImVec2 prevPt{};
    float prevLon = 0.0f;
    for (int i = 0; i <= segments; i++)
    {
      double angle = glm::two_pi<double>() * i / segments;
      glm::dvec3 dir = sub * cosRho + (t1 * std::cos(angle) + t2 * std::sin(angle)) * sinRho;
      OrbitFrames::Geodetic geo = OrbitFrames::eciToGeodeticDeg(dir * OrbitFrames::EARTH_RADIUS_M, thetaGstRad);
      ImVec2 pt = toScreen(static_cast<float>(geo.latDeg), static_cast<float>(geo.lonDeg));
      if (hasPrev && std::abs(static_cast<float>(geo.lonDeg) - prevLon) < 90.0f)
        dl->AddLine(prevPt, pt, IM_COL32(0, 210, 255, 160), 1.2f);
      prevPt = pt;
      prevLon = static_cast<float>(geo.lonDeg);
      hasPrev = true;
    }
  }

  OrbitFrames::Geodetic nowGeo = OrbitFrames::eciToGeodeticDeg(satPosEci, thetaGstRad);
  ImVec2 satPx = toScreen(static_cast<float>(nowGeo.latDeg), static_cast<float>(nowGeo.lonDeg));
  dl->AddCircleFilled(satPx, 5.5f, IM_COL32(0, 220, 255, 255));
  dl->AddCircle(satPx, 7.5f, IM_COL32(255, 255, 255, 200), 16, 1.2f);

  char coordBuf[64];
  std::snprintf(coordBuf, sizeof(coordBuf), "%.2f %s  %.2f %s",
               std::abs(nowGeo.latDeg), nowGeo.latDeg >= 0 ? "N" : "S",
               std::abs(nowGeo.lonDeg), nowGeo.lonDeg >= 0 ? "E" : "W");
  dl->AddText(ImVec2(cp.x + 4.0f, cp.y + cs.y - 16.0f), IM_COL32(220, 220, 220, 220), coordBuf);

  ImGui::EndChild();
}

// Orbit tab: real orbital elements (from the true propagated state, not
// the commanded initial condition -- these drift from the initial
// circular/51.6deg values as J2 acts on the orbit) plus the ground track.
static void drawOrbitTab(const OrbitState &orbitState, const Texture &earthTexture,
                         const std::vector<glm::vec2> &groundTrack, double currentJd)
{
  double mu = TwoBodyGravity{}.mu;
  OrbitalElements elements = OrbitalElements::fromState(orbitState, mu);

  double radiusM = glm::length(orbitState.position);
  double altitudeKm = (radiusM - OrbitFrames::EARTH_RADIUS_M) / 1000.0;
  double apogeeKm = (elements.semiMajorAxisM * (1.0 + elements.eccentricity) - OrbitFrames::EARTH_RADIUS_M) / 1000.0;
  double perigeeKm = (elements.semiMajorAxisM * (1.0 - elements.eccentricity) - OrbitFrames::EARTH_RADIUS_M) / 1000.0;
  double periodMin = elements.periodS(mu) / 60.0;
  double meanMotionRevDay = elements.meanMotionRadS(mu) * 86400.0 / glm::two_pi<double>();

  ImGui::SeparatorText("Orbital State");
  ImGui::TextDisabled("From the real propagated state, not the commanded initial condition --");
  ImGui::TextDisabled("drifts from it as J2 acts on the orbit (see docs/ALGORITHMS.md).");
  ImGui::Text("Altitude: %.2f km", altitudeKm);
  ImGui::Text("Apogee / Perigee altitude: %.2f / %.2f km", apogeeKm, perigeeKm);
  ImGui::Text("Semi-major axis: %.1f km", elements.semiMajorAxisM / 1000.0);
  ImGui::Text("Eccentricity: %.5f", elements.eccentricity);
  ImGui::Text("Inclination: %.3f deg", glm::degrees(elements.inclinationRad));
  ImGui::Text("RAAN: %.3f deg", glm::degrees(elements.raanRad));
  ImGui::Text("Argument of periapsis: %.3f deg", glm::degrees(elements.argPeriapsisRad));
  ImGui::Text("True anomaly: %.3f deg", glm::degrees(elements.trueAnomalyRad));
  ImGui::Text("Period: %.2f min", periodMin);
  ImGui::Text("Mean motion: %.4f rev/day", meanMotionRevDay);

  ImGui::SeparatorText("Ground Track");
  double gmst = OrbitFrames::gmstRad(currentJd);
  drawGroundTrackMinimap(earthTexture, groundTrack, orbitState.position, gmst,
                         glm::radians(static_cast<double>(Config::FOOTPRINT_MIN_ELEVATION_DEG)));
}

static const char *modeName(PointingMode m)
{
  switch (m)
  {
  case PointingMode::NADIR:
    return "NADIR";
  case PointingMode::SUN_POINTING:
    return "SUN_POINTING";
  case PointingMode::DETUMBLE:
    return "DETUMBLE";
  case PointingMode::TARGET:
    return "TARGET";
  case PointingMode::SLEW:
    return "SLEW";
  case PointingMode::FINE_POINTING:
    return "FINE_POINTING";
  case PointingMode::REFLECT:
    return "REFLECT";
  }
  return "?";
}

static const char *controllerName(ControllerType c)
{
  switch (c)
  {
  case ControllerType::PID:
    return "PID";
  case ControllerType::LQR:
    return "LQR";
  case ControllerType::CASCADED:
    return "Cascaded PID";
  }
  return "?";
}

static const char *detumbleActuatorName(DetumbleActuator a)
{
  switch (a)
  {
  case DetumbleActuator::REACTION_WHEELS:
    return "Reaction Wheels";
  case DetumbleActuator::MAGNETORQUERS_BDOT:
    return "Magnetorquers (B-dot)";
  }
  return "?";
}

// Status text/color for one actuator, same priority order and color
// convention the 3D visualization uses (drawReactionWheels/
// drawMagnetorquers): a fault (dead/degraded) always outranks saturation,
// since a degraded unit reporting "saturated" would be misleading about
// what's actually wrong with it.
static void wheelStatus(const ReactionWheel *w, const char *&outText, ImVec4 &outColor)
{
  if (w->healthFactor <= 0.01f)
  {
    outText = "FAILED";
    outColor = ImVec4(1.0f, 0.2f, 0.2f, 1.0f);
  }
  else if (w->healthFactor < 0.99f)
  {
    outText = "DEGRADED";
    outColor = ImVec4(0.85f, 0.1f, 0.85f, 1.0f);
  }
  else if (w->isSaturated())
  {
    outText = "SATURATED";
    outColor = ImVec4(1.0f, 0.4f, 0.0f, 1.0f);
  }
  else
  {
    outText = "OK";
    outColor = ImVec4(0.3f, 1.0f, 0.4f, 1.0f);
  }
}

static void torquerStatus(const Magnetorquer *m, const char *&outText, ImVec4 &outColor)
{
  if (m->healthFactor <= 0.01f)
  {
    outText = "FAILED";
    outColor = ImVec4(1.0f, 0.2f, 0.2f, 1.0f);
  }
  else if (m->healthFactor < 0.99f)
  {
    outText = "DEGRADED";
    outColor = ImVec4(0.85f, 0.1f, 0.85f, 1.0f);
  }
  else if (std::abs(m->getSaturationRatio()) >= 0.99f)
  {
    outText = "SATURATED";
    outColor = ImVec4(1.0f, 0.4f, 0.0f, 1.0f);
  }
  else
  {
    outText = "OK";
    outColor = ImVec4(0.3f, 1.0f, 0.4f, 1.0f);
  }
}

static void drawFswTab(ADCS &adcs, SensorTelemetry &telemetry, float trueErrDeg)
{
  static const char *modeNames[] = {"Nadir", "Sun Pointing", "Detumble", "Target", "Slew", "Fine Pointing", "Reflect"};
  int modeIdx = static_cast<int>(adcs.mode);
  if (ImGui::Combo("Pointing mode", &modeIdx, modeNames, IM_ARRAYSIZE(modeNames)))
    adcs.mode = static_cast<PointingMode>(modeIdx);
  ImGui::TextDisabled("This is what's commanded -- FDIR can override it (see FDIR tab).");

  if (adcs.mode != adcs.effectiveMode)
    ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.2f, 1.0f), "FDIR override active: flying %s instead", modeName(adcs.effectiveMode));

  if (adcs.manualOverride)
    ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.2f, 1.0f), "Manual actuator override is active (Actuators tab) -- FSW is not commanding hardware.");

  ImGui::SeparatorText("Pointing Error");
  if (adcs.mode == PointingMode::DETUMBLE)
    ImGui::TextDisabled("DETUMBLE has no attitude target; showing the last value from before it was entered.");
  ImGui::Text("FSW estimate: %.2f deg", adcs.estimatedPointingErrorDeg);
  plotChannel("Estimated error", telemetry.estimatedPointingErrorDeg, "deg");
  ImGui::Text("True (ground truth, diagnostic only): %.2f deg", trueErrDeg);
  plotChannel("True error", telemetry.truePointingErrorDeg, "deg");
  ImGui::Text("Estimator confidence (1-sigma): %.4f deg (%.1f arcsec)",
              adcs.attitudeUncertaintyDeg, adcs.attitudeUncertaintyDeg * 3600.0f);
  ImGui::TextDisabled("Grows during a star-tracker dropout, shrinks once a correction lands again.");

  ImGui::SeparatorText("Attitude Controller");
  if (adcs.mode == PointingMode::DETUMBLE)
    ImGui::TextDisabled("DETUMBLE ignores this and uses the Detumble Actuator law below instead.");

  static const char *controllerNames[] = {"PID", "LQR", "Cascaded PID"};
  int controllerIdx = static_cast<int>(adcs.controllerType);
  if (ImGui::Combo("Algorithm", &controllerIdx, controllerNames, IM_ARRAYSIZE(controllerNames)))
    adcs.controllerType = static_cast<ControllerType>(controllerIdx);

  switch (adcs.controllerType)
  {
  case ControllerType::PID:
  {
    PIDController &c = adcs.pidController();
    ImGui::DragFloat("Kp", &c.Kp, 0.0001f, 0.0f, 1.0f, "%.5f");
    ImGui::DragFloat("Ki", &c.Ki, 0.00001f, 0.0f, 0.1f, "%.6f");
    ImGui::DragFloat("Kd", &c.Kd, 0.0001f, 0.0f, 1.0f, "%.5f");
    ImGui::DragFloat("Max integral", &c.maxIntegral, 0.001f, 0.0f, 1.0f, "%.4f");
    break;
  }
  case ControllerType::LQR:
  {
    LQRController &c = adcs.lqrController();
    ImGui::DragFloat3("Kp (x,y,z)", &c.Kp.x, 0.0001f, 0.0f, 1.0f, "%.5f");
    ImGui::DragFloat3("Kd (x,y,z)", &c.Kd.x, 0.0001f, 0.0f, 1.0f, "%.5f");
    ImGui::DragFloat("Omega max (rad/s)", &c.omega_max, 0.01f, 0.01f, 5.0f, "%.3f");
    ImGui::TextDisabled("Q/R weights (derived by auto-tune, view only):");
    ImGui::Text("Q_att:  %.4f  %.4f  %.4f", c.Q_att.x, c.Q_att.y, c.Q_att.z);
    ImGui::Text("Q_rate: %.4f  %.4f  %.4f", c.Q_rate.x, c.Q_rate.y, c.Q_rate.z);
    ImGui::Text("R:      %.4f  %.4f  %.4f", c.R.x, c.R.y, c.R.z);
    break;
  }
  case ControllerType::CASCADED:
  {
    CascadedController &c = adcs.cascadedController();
    ImGui::DragFloat("Settling time (s)", &c.settlingTime, 0.1f, 0.5f, 30.0f, "%.2f");
    ImGui::DragFloat("Damping ratio", &c.dampingRatio, 0.01f, 0.1f, 3.0f, "%.3f");
    ImGui::DragFloat("Omega max (rad/s)", &c.omega_max, 0.01f, 0.01f, 5.0f, "%.3f");
    break;
  }
  }
  if (ImGui::Button("Reset to auto-tuned gains for current mode"))
    adcs.retuneForMode();

  ImGui::SeparatorText("Detumble Actuator");
  static const char *detumbleNames[] = {"Reaction Wheels", "Magnetorquers (B-dot)"};
  int detumbleIdx = static_cast<int>(adcs.detumbleActuator);
  if (ImGui::Combo("Detumble via", &detumbleIdx, detumbleNames, IM_ARRAYSIZE(detumbleNames)))
    adcs.detumbleActuator = static_cast<DetumbleActuator>(detumbleIdx);

  if (adcs.detumbleActuator == DetumbleActuator::MAGNETORQUERS_BDOT)
  {
    ImGui::DragFloat("B-dot gain (A*m^2 per T/s)", &adcs.bdotGain, 100.0f, 0.0f, 1.0e7f, "%.0f");
    ImGui::Text("dB/dt (body): %.2f uT/s -- see Sensors tab for the field itself", glm::length(adcs.magFieldRateBody) * 1e6f);
  }
}

static void drawSensorsTab(ADCS &adcs, SensorTelemetry &telemetry)
{
  ImGui::SeparatorText("IMU");
  glm::vec3 gyroDeg = glm::degrees(adcs.lastGyroBody);
  ImGui::Text("Gyro (deg/s):  %+7.2f  %+7.2f  %+7.2f", gyroDeg.x, gyroDeg.y, gyroDeg.z);
  plotChannel("Gyro rate", telemetry.gyroMagDegS, "deg/s");
  ImGui::Text("Accel (m/s^2): %+7.3f  %+7.3f  %+7.3f", adcs.lastAccelBody.x, adcs.lastAccelBody.y, adcs.lastAccelBody.z);
  plotChannel("Accel", telemetry.accelMagMs2, "m/s^2");

  ImGui::SeparatorText("Magnetometer");
  glm::vec3 fieldUt = adcs.magFieldBody * 1e6f;
  ImGui::Text("Field (uT): %+7.2f  %+7.2f  %+7.2f", fieldUt.x, fieldUt.y, fieldUt.z);
  plotChannel("Field magnitude", telemetry.magFieldMagUt, "uT");
  ImGui::Text("dB/dt: %.2f uT/s", glm::length(adcs.magFieldRateBody) * 1e6f);

  ImGui::SeparatorText("Star Tracker");
  if (adcs.starTrackerValid)
    ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.4f, 1.0f), "[VALID] -- primary attitude correction");
  else if (adcs.triadFallbackUsed)
    ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1.0f), "[BLINDED/SLEWING] -- falling back to sun+magnetometer TRIAD");
  else
    ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.2f, 1.0f), "[NO CORRECTION] -- coasting on gyro propagation only");
  ImGui::TextDisabled("Blinded when the boresight comes within the sun-exclusion angle, or while slewing too fast to centroid stars.");
}

static void drawActuatorsTab(ADCS &adcs, const std::vector<ReactionWheel *> &wheels,
                             const std::vector<Magnetorquer *> &magnetorquers)
{
  ImGui::SeparatorText("Reaction Wheels");
  for (size_t i = 0; i < wheels.size(); i++)
  {
    const ReactionWheel *w = wheels[i];
    const char *statusText;
    ImVec4 color;
    wheelStatus(w, statusText, color);

    float rpm = w->currentSpeed * 60.0f / (2.0f * glm::pi<float>());
    ImGui::Text("Wheel %zu", i);
    ImGui::SameLine();
    ImGui::TextColored(color, "[%s]", statusText);
    ImGui::Text("  cmd: %+.5f Nm   speed: %+.0f RPM   sat: %.0f%%",
                w->commandedTorque, rpm, std::abs(w->getSaturationRatio()) * 100.0f);
  }

  ImGui::SeparatorText("Magnetorquers");
  for (size_t i = 0; i < magnetorquers.size(); i++)
  {
    const Magnetorquer *m = magnetorquers[i];
    const char *statusText;
    ImVec4 color;
    torquerStatus(m, statusText, color);

    ImGui::Text("Rod %zu", i);
    ImGui::SameLine();
    ImGui::TextColored(color, "[%s]", statusText);
    ImGui::Text("  cmd: %+.3f A*m^2   sat: %.0f%%",
                m->commandedDipoleMoment, std::abs(m->getSaturationRatio()) * 100.0f);
  }

  ImGui::SeparatorText("Manual Control");
  ImGui::Checkbox("Enable manual override", &adcs.manualOverride);
  if (adcs.manualOverride)
  {
    ImGui::TextDisabled("Directly commands hardware; FSW guidance/control/allocation is skipped.");

    for (size_t i = 0; i < wheels.size() && i < (size_t)NUM_WHEELS; i++)
    {
      char label[32];
      std::snprintf(label, sizeof(label), "Wheel %zu (Nm)", i);
      ImGui::SliderFloat(label, &adcs.manualWheelTorqueNm[i], -wheels[i]->maxTorque, wheels[i]->maxTorque, "%.5f");
    }

    for (size_t i = 0; i < magnetorquers.size() && i < (size_t)NUM_TORQUERS; i++)
    {
      char label[32];
      std::snprintf(label, sizeof(label), "Rod %zu (A*m^2)", i);
      ImGui::SliderFloat(label, &adcs.manualMagnetorquerMomentAm2[i], -magnetorquers[i]->maxDipoleMoment, magnetorquers[i]->maxDipoleMoment, "%.3f");
    }
  }
}

static const char *fdirFaultName(uint32_t flag)
{
  switch (flag)
  {
  case FDIR_FAULT_WHEEL_AUTHORITY_LOST:
    return "Wheel authority lost";
  case FDIR_FAULT_ATTITUDE_UNCERTAIN:
    return "Attitude uncertain";
  case FDIR_FAULT_EXCESS_RATE:
    return "Excess body rate";
  case FDIR_FAULT_LOW_BATTERY:
    return "Low battery";
  default:
    return "Unknown";
  }
}

// Renders `flags` as a comma-separated list of fault names into `out`
// (fixed-size, matching this project's no-dynamic-allocation-in-FSW-UI
// convention already used elsewhere in this file for label buffers).
static void formatFaultFlags(uint32_t flags, char *out, size_t outSize)
{
  out[0] = '\0';
  bool first = true;
  for (uint32_t bit = 1; bit != 0; bit <<= 1)
  {
    if (!(flags & bit))
      continue;
    size_t used = std::strlen(out);
    std::snprintf(out + used, outSize - used, "%s%s", first ? "" : ", ", fdirFaultName(bit));
    first = false;
  }
}

// Autonomous mode-manager/FDIR status: what state it's in, which faults are
// currently latched, the tunable thresholds that decide when a fault trips,
// and a scrollable log of every trip/clear event -- the flight-software
// equivalent of a fault log a real ops team would review after the fact.
static void drawFdirTab(ADCS &adcs)
{
  FDIR &fdir = adcs.fdir;

  ImGui::SeparatorText("Status");
  if (fdir.state() == FdirState::NOMINAL)
    ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.4f, 1.0f), "NOMINAL");
  else
    ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.2f, 1.0f), "SAFE_HOLD");

  ImGui::Checkbox("Autonomy enabled", &fdir.enabled);
  ImGui::TextDisabled("When off, faults are still detected and logged but never override the commanded mode.");

  ImGui::SeparatorText("Active Faults");
  uint32_t active = fdir.activeFaults();
  if (active == FDIR_FAULT_NONE)
  {
    ImGui::TextDisabled("None");
  }
  else
  {
    for (uint32_t bit = 1; bit != 0; bit <<= 1)
      if (active & bit)
        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.2f, 1.0f), "[%s]", fdirFaultName(bit));
  }
  if (ImGui::Button("Clear latched faults (ground command)"))
    fdir.clearLatchedFaults();
  ImGui::TextDisabled("Faults latch on trip and stay active even if the condition clears -- this is the explicit ack that drops them.");

  ImGui::SeparatorText("Thresholds");
  ImGui::DragInt("Min healthy wheels", &fdir.minHealthyWheels, 0.05f, 0, NUM_WHEELS);
  ImGui::DragFloat("Uncertainty trigger (deg)", &fdir.attitudeUncertaintyTriggerDeg, 0.1f, 0.1f, 45.0f, "%.1f");
  ImGui::DragFloat("Uncertainty sustain (s)", &fdir.attitudeUncertaintySustainedS, 0.5f, 0.0f, 60.0f, "%.1f");
  ImGui::DragFloat("Excess rate (rad/s)", &fdir.excessRateRadS, 0.05f, 0.1f, 10.0f, "%.2f");
  ImGui::DragFloat("Low battery trigger (fraction)", &fdir.lowBatterySocTrigger, 0.01f, 0.0f, 1.0f, "%.2f");

  ImGui::SeparatorText("Event Log (newest first)");
  if (fdir.eventCount == 0)
    ImGui::TextDisabled("No events yet.");
  for (int i = 0; i < fdir.eventCount; i++)
  {
    const FdirEvent &ev = fdir.recentEvent(i);
    char flagsBuf[128];
    formatFaultFlags(ev.flags, flagsBuf, sizeof(flagsBuf));
    ImVec4 color = ev.entering ? ImVec4(1.0f, 0.4f, 0.2f, 1.0f) : ImVec4(0.5f, 0.8f, 1.0f, 1.0f);
    ImGui::TextColored(color, "[t=%7.1fs] %s: %s", ev.missionTimeS, ev.entering ? "TRIPPED" : "CLEARED", flagsBuf);
  }
}

// EPS (electrical power subsystem): battery state, live solar generation
// per panel, and the same net-power/SOC plots the FSW-cycle telemetry
// below is pushed to -- the electrical equivalent of the Sensors/Actuators
// tabs, showing what's actually happening on the bus rather than what FSW
// perceives (there's no EPS "sensor model" with its own noise/dropouts in
// this project -- battery telemetry is read directly, see PowerSample's
// header comment).
static void drawEpsTab(Cubesat &sat, ADCS &adcs, SensorTelemetry &telemetry, bool inEclipse)
{
  float soc = sat.battery.stateOfCharge();
  ImVec4 socColor = soc > 0.5f   ? ImVec4(0.3f, 1.0f, 0.4f, 1.0f)
                    : soc > 0.2f ? ImVec4(1.0f, 0.8f, 0.2f, 1.0f)
                                 : ImVec4(1.0f, 0.3f, 0.2f, 1.0f);

  ImGui::SeparatorText("Battery");
  ImGui::TextColored(socColor, "State of charge: %.1f%%", soc * 100.0f);
  plotChannel("State of charge", telemetry.batterySocPct, "%");
  ImGui::Text("Voltage: %.2f V", sat.battery.voltage());
  ImGui::Text("Energy: %.1f / %.1f Wh", sat.battery.energyJ / 3600.0f, sat.battery.capacityJ / 3600.0f);
  ImGui::Text("Net power: %.2f W", telemetry.netPowerW.last());
  plotChannel("Net power", telemetry.netPowerW, "W");
  if (sat.battery.isDepleted())
    ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.2f, 1.0f), "DEPLETED");

  ImGui::TextDisabled("Testing:");
  ImGui::SameLine();
  if (ImGui::Button("Drain to 15%"))
    sat.battery.energyJ = 0.15f * sat.battery.capacityJ;
  ImGui::SameLine();
  if (ImGui::Button("Full charge"))
    sat.battery.energyJ = sat.battery.capacityJ;

  ImGui::SeparatorText("Solar Panels");
  glm::vec3 sunDirWorld = adcs.sunPosition - sat.body->position;
  float totalGenW = 0.0f;
  static const char *panelNames[6] = {"+X", "-X", "+Y", "-Y", "+Z", "-Z"};
  for (size_t i = 0; i < sat.solarPanels.size(); i++)
  {
    SolarPanel::Reading r = sat.solarPanels[i].sample(*sat.body, sunDirWorld, Config::SOLAR_FLUX_WM2);
    float panelPowerW = inEclipse ? 0.0f : r.powerW;
    totalGenW += panelPowerW;
    const char *name = i < 6 ? panelNames[i] : "?";
    ImGui::Text("%s face: %5.2f W  (incidence %5.1f deg)", name, panelPowerW, r.incidenceAngleDeg);
  }
  ImGui::Text("Total generation: %.2f W", totalGenW);
  if (inEclipse)
    ImGui::TextColored(ImVec4(0.6f, 0.6f, 0.7f, 1.0f), "In eclipse -- Earth's shadow, no generation.");
  else
    ImGui::TextDisabled("Sunlit.");
}

// Simulation-level knobs, as opposed to ADCS/FSW state -- things that
// belong to the test harness around the spacecraft, not to the spacecraft
// itself. Held by value in main() and handed to drawSimulationTab() by
// reference each frame, same pattern as ADCS's own public fields.
struct SimControls
{
  bool paused = false;
  float tumbleKickRadS;

  explicit SimControls(float tumbleKickRadSIn) : tumbleKickRadS(tumbleKickRadSIn) {}
};

// Mission epoch (UTC calendar date/time that orbitState.missionTimeS = 0
// corresponds to), editable live from the Simulation tab -- changing it
// doesn't touch the orbit itself (orbitState keeps propagating exactly as
// it was), only what real Julian Date main()'s currentJdNow maps
// missionTimeS onto, which is what Sun direction (SunModel) and Earth's
// rotation (OrbitFrames::gmstRad, see drawEarth) are actually computed
// from -- so this is genuinely "what historical/future date is this
// orbit happening on," not a simulation reset.
struct EpochControls
{
  int year = 2026;
  int month = 1;
  int day = 1;
  int hour = 0;
  int minute = 0;
  float second = 0.0f;
};

static void drawSimulationTab(SimControls &sim, EpochControls &epoch,
                              std::vector<ReactionWheel *> &wheels, RigidBody *body, ADCS &adcs)
{
  ImGui::SeparatorText("Simulation");
  ImGui::Checkbox("Pause simulation", &sim.paused);
  ImGui::TextDisabled("Physics and FSW freeze; camera/UI stay live.");

  ImGui::SeparatorText("Mission Epoch (UTC)");
  ImGui::TextDisabled("What real calendar date orbitState's mission clock started at --");
  ImGui::TextDisabled("Sun direction and Earth's rendered rotation both follow this live.");
  int ymd[3] = {epoch.year, epoch.month, epoch.day};
  if (ImGui::InputInt3("Year / Month / Day", ymd))
  {
    epoch.year = ymd[0];
    epoch.month = glm::clamp(ymd[1], 1, 12);
    epoch.day = glm::clamp(ymd[2], 1, 31);
  }
  int hms[2] = {epoch.hour, epoch.minute};
  if (ImGui::InputInt2("Hour / Minute", hms))
  {
    epoch.hour = glm::clamp(hms[0], 0, 23);
    epoch.minute = glm::clamp(hms[1], 0, 59);
  }
  ImGui::DragFloat("Second", &epoch.second, 1.0f, 0.0f, 59.999f, "%.3f", ImGuiSliderFlags_AlwaysClamp);

  ImGui::SeparatorText("Disturbances");
  ImGui::DragFloat("Tumble kick (rad/s)", &sim.tumbleKickRadS, 0.05f, 0.0f, 5.0f, "%.2f");
  if (ImGui::Button("Kick into random tumble [T]"))
  {
    static std::mt19937 tumbleRng(std::random_device{}());
    std::uniform_real_distribution<float> d(-sim.tumbleKickRadS, sim.tumbleKickRadS);
    body->angularVelocity = glm::vec3(d(tumbleRng), d(tumbleRng), d(tumbleRng));
  }

  ImGui::SeparatorText("Reaction Wheel Faults");
  ImGui::TextDisabled("Manual only -- pick a wheel and a fault, same healthFactor model FDIR reacts to.");
  static float degradeSeverity = 0.3f; // shared by every "Degrade" button below
  ImGui::SliderFloat("Degrade severity (healthFactor)", &degradeSeverity, 0.05f, 0.95f, "%.2f");
  for (size_t i = 0; i < wheels.size(); i++)
  {
    ImGui::PushID(static_cast<int>(i));
    ImGui::Text("Wheel %zu (health %.0f%%)", i, wheels[i]->healthFactor * 100.0f);
    ImGui::SameLine();
    if (ImGui::SmallButton("Fail"))
      wheels[i]->healthFactor = 0.0f;
    ImGui::SameLine();
    if (ImGui::SmallButton("Degrade"))
      wheels[i]->healthFactor = degradeSeverity;
    ImGui::SameLine();
    if (ImGui::SmallButton("Repair"))
      wheels[i]->healthFactor = 1.0f;
    ImGui::PopID();
  }
  if (ImGui::Button("Repair all wheels"))
    for (auto *w : wheels)
      w->healthFactor = 1.0f;

  ImGui::SeparatorText("Momentum Desaturation");
  float maxWheelSat = 0.0f;
  for (auto *w : wheels)
    maxWheelSat = std::max(maxWheelSat, std::abs(w->getSaturationRatio()));
  ImGui::Text("Peak wheel saturation: %.0f%%", maxWheelSat * 100.0f);
  if (adcs.desatActive)
    ImGui::TextColored(ImVec4(0.2f, 0.7f, 1.0f, 1.0f), "Desaturating via magnetorquers...");
  else
    ImGui::TextDisabled("Idle");

  if (ImGui::Button("Desaturate Wheels Now"))
    adcs.requestDesaturation();
  ImGui::SameLine();
  ImGui::TextDisabled("(runs in the background until wheel momentum is low; keeps pointing)");

  ImGui::Checkbox("Auto-desaturate when a wheel gets close to saturated", &adcs.desatAutoTriggerEnabled);
  ImGui::DragFloat("Auto-trigger threshold (fraction)", &adcs.desatTriggerSaturation, 0.01f, 0.5f, 1.0f, "%.2f", ImGuiSliderFlags_AlwaysClamp);
  ImGui::DragFloat("Auto-stop threshold (fraction)", &adcs.desatStopSaturation, 0.01f, 0.0f, 0.9f, "%.2f", ImGuiSliderFlags_AlwaysClamp);
  ImGui::DragFloat("Desaturation gain", &adcs.desatGain, 0.001f, 0.0f, 1.0f, "%.4f");
}

// Single ADCS panel, organized as tabs instead of separate windows so the
// whole flight-software state lives in one place: FSW (pointing mode,
// attitude algorithm + gains, detumble actuator + B-dot gain -- everything
// that decides *what* to command), Sensors (every sensor's current reading
// plus a rolling plot -- what the FSW actually perceives), Actuators
// (every actuator's current command + health/saturation status, plus the
// manual-override controls), and Simulation (test-harness controls that
// aren't part of the spacecraft itself: pause, mission epoch, induced
// tumbles, and manual reaction-wheel fault triggers).
static void drawADCSPanel(ADCS &adcs, Cubesat &sat,
                          SensorTelemetry &telemetry, SimControls &sim,
                          EpochControls &epoch,
                          const OrbitState &orbitState, const Texture &earthTexture,
                          const std::vector<glm::vec2> &groundTrack, double currentJd,
                          float trueErrDeg, bool inEclipse)
{
  ImGui::SetNextWindowPos(ImVec2(20, 20), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSize(ImVec2(380, 560), ImGuiCond_FirstUseEver);
  ImGui::Begin("CubeSat ADCS");

  if (ImGui::BeginTabBar("ADCSTabs"))
  {
    if (ImGui::BeginTabItem("FSW"))
    {
      drawFswTab(adcs, telemetry, trueErrDeg);
      ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Sensors"))
    {
      drawSensorsTab(adcs, telemetry);
      ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Actuators"))
    {
      drawActuatorsTab(adcs, sat.wheels, sat.magnetorquers);
      ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("FDIR"))
    {
      drawFdirTab(adcs);
      ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("EPS"))
    {
      drawEpsTab(sat, adcs, telemetry, inEclipse);
      ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Orbit"))
    {
      drawOrbitTab(orbitState, earthTexture, groundTrack, currentJd);
      ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Simulation"))
    {
      drawSimulationTab(sim, epoch, sat.wheels, sat.body, adcs);
      ImGui::EndTabItem();
    }
    ImGui::EndTabBar();
  }

  ImGui::End();
}

static void updateTitle(GLFWwindow *win, PointingMode mode, float attErrDeg,
                        float rateRadS, int faultedWheels)
{
  char buf[256];
  std::snprintf(buf, sizeof(buf),
                "CubeSat ADCS (Pyramid RWA)  |  [1-6] Mode  [Space] Target  [T] Tumble  |  "
                "Mode: %s  |  Att err: %.1f deg  |  Rate: %.3f rad/s  |  Faulted wheels: %d",
                modeName(mode), attErrDeg, rateRadS, faultedWheels);
  glfwSetWindowTitle(win, buf);
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main()
{
  GUI gui(800, 600, "CubeSat ADCS (Pyramid RWA)");
  ImGuiLayer imguiLayer(gui);
  gui.camera
      .setUp({0, 0, 1})
      .setClipPlanes(Config::CAMERA_NEAR, Config::CAMERA_FAR)
      .setFOV(Config::CAMERA_FOV);
  // The scene now spans a 0.1m cubesat up to a ~6.9e6m orbital radius --
  // a standard depth buffer can't hold that dynamic range without
  // z-fighting (near:far is on the order of 1e9). Logarithmic depth
  // trades a small amount of precision at extreme distances for correct
  // ordering across the whole range.
  gui.setLogDepth(Config::CAMERA_FAR);

  // Loaded after the GL context is live (Texture::loadFromFile requires
  // it) -- falls back to a grey 1x1 texture with a printed warning if the
  // file isn't found, so a missing asset doesn't crash the sim.
  Texture earthTexture;
  earthTexture.loadFromFile(std::string(RESOURCES_DIR) + "/textures/earth.jpg");

  PhysicsWorld world;
  HardwareConfig hwConfig;
  Cubesat sat = buildCubesatPyramid(world, hwConfig);

  // Built after the satellite so the orbit target can start on its actual
  // position rather than a magic-number guess at where buildCubesatPyramid()
  // happens to place it.
  OrbitalCamera orbit(
      Config::CAMERA_INITIAL_DISTANCE,
      45.0f, 0.0f,
      sat.body->position);
  orbit.setMaxDistance(Config::CAMERA_MAX_DISTANCE)
      .setMinDistance(Config::CAMERA_MIN_DISTANCE)
      .setZoomSensitivity(Config::ZOOM_SENSITIVITY)
      .setPanSensitivity(Config::PAN_SENSITIVITY);

  glm::vec2 lastMousePos = gui.getMousePosition();

  // Flight software: ADCS is hardware-abstracted (see FlightTypes.h) --
  // configure() gives it a fixed hardware description and an initial
  // attitude estimate; every cycle after this, step() only ever sees
  // plain FSWInputs the harness (this loop) builds from the simulated
  // sensors, and returns plain FSWOutputs the harness applies to the
  // simulated actuators. ADCS itself never holds a RigidBody*/sensor
  // pointer/actuator pointer at all.
  ADCS adcs;
  adcs.configure(hwConfig, sat.body->orientation);
  adcs.target = randomTarget();
  float adcsTimer = 0.0f;
  float missionTime = 0.0f;
  float trueErrDeg = 0.0f; // harness-side diagnostic: true (ground-truth) pointing error, held between ADCS cycles like fieldNow below

  // No Gravity generator is attached to this world (free-floating orbit),
  // so ambient gravity for the IMU model is zero -- this is harness-side
  // simulation setup now, not something ADCS itself ever sees or assumes.
  glm::vec3 gravity{0.0f};

  // Real orbital truth: double-precision ECI state (see rigidbody/orbit/
  // OrbitState.h), propagated independently of sat.body's own float32
  // scene-frame position -- this is genuinely a *different* position, not
  // a rescaled version of it (see FlightTypes.h's spacecraftPositionWorld
  // comment). Two-body + J2, ISS-like default (500km circular, 51.6deg
  // inclination) -- the same altitude/inclination the old kinematic
  // magnetic-field stand-in used, now driving real position, sun
  // direction, eclipse state, and magnetic field.
  OrbitState orbitState = OrbitalElements::circular(
                              500e3, glm::radians(51.6), OrbitFrames::EARTH_RADIUS_M)
                              .toState(TwoBodyGravity{}.mu);
  OrbitPropagator orbitPropagator;
  orbitPropagator.addForceModel(std::make_unique<TwoBodyGravity>());
  orbitPropagator.addForceModel(std::make_unique<J2Perturbation>());
  EpochControls epoch; // editable live from the Simulation tab -- see its own comment
  double missionEpochJd = OrbitTime::julianDate(epoch.year, epoch.month, epoch.day, epoch.hour, epoch.minute, epoch.second);

  // Real tilted-dipole field, sampled at orbitState's true position each
  // frame (see below) -- replaces the old MagneticField's fake kinematic
  // orbit phase with the real one. Written into both the magnetorquers
  // (which need it to turn a commanded dipole moment into torque) and
  // ADCS's ambientFieldWorld (its TRIAD reference vector), the same way
  // `gravity` feeds the IMU sample below.
  CentralBodyMagneticField magField;

  SensorTelemetry telemetry(Config::TELEMETRY_HISTORY_SAMPLES);

  SimControls sim(Config::TUMBLE_KICK_RAD_S);
  glm::vec3 fieldNow{0.0f};                 // last-sampled ambient field; held while paused rather than resampled
  bool inEclipse = false;                   // last-computed shadow state; held while paused
  glm::vec3 earthRelativePositionNow{0.0f}; // orbitState.position cast to float; held while paused

  // Predicted orbit path (render-scale points, see computePredictedOrbitPath)
  // -- computed once up front and refreshed periodically, not every frame.
  std::vector<glm::vec3> orbitPathPoints = computePredictedOrbitPath(orbitState, Config::ORBIT_PATH_POINTS);
  float orbitPathRefreshTimer = 0.0f;
  double currentJdNow = missionEpochJd; // held while paused, and for drawEarth() below

  // Ground track: (lat, lon) history in degrees, sampled periodically (see
  // Config::GROUND_TRACK_SAMPLE_INTERVAL_S) rather than every frame, capped
  // at Config::GROUND_TRACK_MAX_POINTS so it spans multiple orbits without
  // growing unbounded over a long-running mission.
  std::vector<glm::vec2> groundTrackLatLonDeg;
  float groundTrackSampleTimer = 0.0f;

  float lastTime = glfwGetTime();
  while (!gui.shouldClose())
  {
    float time = glfwGetTime();
    float dt = time - lastTime;
    lastTime = time;

    // =================== INPUT ===================
    glm::vec2 mousePos = gui.getMousePosition();
    glm::vec2 mouseDelta = mousePos - lastMousePos;
    lastMousePos = mousePos;

    if (gui.isKeyJustPressed(GLFW_KEY_1))
      adcs.mode = PointingMode::NADIR;
    if (gui.isKeyJustPressed(GLFW_KEY_2))
      adcs.mode = PointingMode::SUN_POINTING;
    if (gui.isKeyJustPressed(GLFW_KEY_3))
      adcs.mode = PointingMode::DETUMBLE;
    if (gui.isKeyJustPressed(GLFW_KEY_4))
      adcs.mode = PointingMode::TARGET;
    if (gui.isKeyJustPressed(GLFW_KEY_5))
      adcs.mode = PointingMode::SLEW;
    if (gui.isKeyJustPressed(GLFW_KEY_6))
      adcs.mode = PointingMode::FINE_POINTING;
    if (gui.isKeyJustPressed(GLFW_KEY_7))
      adcs.mode = PointingMode::REFLECT;

    if (gui.isKeyJustPressed(GLFW_KEY_SPACE))
    {
      adcs.target = randomTarget();
      adcs.resetController(); // clear integral windup from previous target
    }

    if (gui.isKeyJustPressed(GLFW_KEY_T))
    {
      static std::mt19937 tumbleRng(std::random_device{}());
      std::uniform_real_distribution<float> tumbleDist(-sim.tumbleKickRadS, sim.tumbleKickRadS);
      sat.body->angularVelocity = glm::vec3(tumbleDist(tumbleRng), tumbleDist(tumbleRng), tumbleDist(tumbleRng));
    }

    // Follow the satellite: recenter the orbit camera's target on its
    // real current position every frame (like constellation-sim does for
    // a selected satellite), so free yaw/pitch/zoom around it still work
    // (setTarget doesn't touch distance/yaw/pitch) while any right-drag
    // pan gets overridden next frame -- a locked-on chase camera can't
    // also be panned away from what it's locked onto.
    orbit.setTarget(sat.body->position);

    // Don't drive the orbit camera from mouse input ImGui itself wants
    // (e.g. dragging the Controller window around) -- otherwise moving a
    // panel also spins the camera underneath it.
    if (!ImGui::GetIO().WantCaptureMouse)
      orbit.handleInput(gui, mouseDelta, gui.getScrollDelta());
    orbit.applyToCamera(gui.camera);

    // =================== SIMULATION ===================
    // Everything below (mission clock, field sampling, FSW, fault
    // injection, physics) freezes while paused; camera/UI/mode selection
    // above stay live so the panel is still usable mid-pause.
    if (!sim.paused)
    {
      missionTime += dt;

      // Real orbital truth, propagated every frame at the frame's own dt
      // (RK4 is stable at any step size -- smaller just means more calls,
      // not less accuracy) -- integrated independently of
      // PhysicsWorld::step() below in double precision, then bridged into
      // sat.body->position as a single non-accumulating cast every frame
      // (see OrbitState.h's header comment for why the double-precision
      // integration itself has to stay separate from RigidBody's float32
      // state, even though the *result* is copied into it here). Because
      // this is a fresh cast from the true state each frame rather than
      // something PhysicsWorld itself integrates, there's no float32
      // accumulation risk -- sat.body->velocity is deliberately never set
      // to match, so PhysicsWorld's own (float32) translational
      // integration contributes nothing on top of this.
      orbitPropagator.step(orbitState, dt);
      // Recomputed from the Simulation tab's live-editable fields every
      // frame -- cheap (six ints/one float into a JD formula), and means
      // an edit takes effect immediately without a separate "apply" step.
      missionEpochJd = OrbitTime::julianDate(epoch.year, epoch.month, epoch.day, epoch.hour, epoch.minute, epoch.second);
      currentJdNow = OrbitTime::advance(missionEpochJd, orbitState.missionTimeS);
      earthRelativePositionNow = glm::vec3(orbitState.position); // single non-accumulating cast -- see OrbitState.h
      sat.body->position = earthRelativePositionNow;
      glm::dvec3 sunDirEci = SunModel::directionEci(currentJdNow);
      inEclipse = EclipseModel::inEclipse(orbitState.position, sunDirEci);

      // Refresh the predicted-path polyline periodically (not every
      // frame -- see ORBIT_PATH_REFRESH_S) since it only drifts slowly
      // (mainly from J2) cycle to cycle.
      orbitPathRefreshTimer += dt;
      if (orbitPathRefreshTimer > Config::ORBIT_PATH_REFRESH_S)
      {
        orbitPathPoints = computePredictedOrbitPath(orbitState, Config::ORBIT_PATH_POINTS);
        orbitPathRefreshTimer = 0.0f;
      }

      // Sample the ground track periodically (not every frame -- see
      // GROUND_TRACK_SAMPLE_INTERVAL_S).
      groundTrackSampleTimer += dt;
      if (groundTrackSampleTimer > Config::GROUND_TRACK_SAMPLE_INTERVAL_S)
      {
        OrbitFrames::Geodetic geo = OrbitFrames::eciToGeodeticDeg(orbitState.position, OrbitFrames::gmstRad(currentJdNow));
        groundTrackLatLonDeg.push_back(glm::vec2(static_cast<float>(geo.latDeg), static_cast<float>(geo.lonDeg)));
        if (static_cast<int>(groundTrackLatLonDeg.size()) > Config::GROUND_TRACK_MAX_POINTS)
          groundTrackLatLonDeg.erase(groundTrackLatLonDeg.begin());
        groundTrackSampleTimer = 0.0f;
      }

      // Sun position for guidance/EPS: the real ECI position (~1.5e11 m),
      // not an arbitrary nearby offset -- now that sat.body->position is
      // real too (~6.9e6 m), placing the sun a small fixed distance away
      // (the old "2 units from the satellite" convention) would suffer
      // the same catastrophic-cancellation problem randomTarget()'s
      // comment describes: adding a small offset to spacecraftPositionWorld
      // and then subtracting it back out to recover a direction loses
      // almost all of that offset's precision once the base position's
      // magnitude dwarfs it. The real Sun position doesn't have this
      // problem -- it's the *dominant* term in sunPosition -
      // spacecraftPositionWorld, not a small perturbation of it, so the
      // subtraction stays well-conditioned.
      adcs.sunPosition = glm::vec3(SunModel::positionEci(currentJdNow));

      // Ambient field at the satellite's real orbital position -- fed to
      // the magnetorquers (they need it every physics substep to turn a
      // commanded dipole moment into torque) and to ADCS (it needs it to
      // interpret the magnetometer), same role adcs.gravity plays for the
      // IMU.
      fieldNow = magField.sample(earthRelativePositionNow);
      for (auto *rod : sat.magnetorquers)
        rod->ambientFieldWorld = fieldNow;
      adcs.ambientFieldWorld = fieldNow;

      // =================== FLIGHT SOFTWARE (20 Hz) ===================
      // This is the HAL boundary in action: sample every simulated sensor,
      // pack the readings into a plain FSWInputs, hand it to ADCS::step()
      // (which never sees a RigidBody/sensor/actuator object at all), then
      // apply the plain FSWOutputs it returns to the simulated actuators.
      // A HIL adapter would replace exactly the sampling and command-
      // application on either side of that call -- ADCS::step() itself
      // wouldn't change.
      adcsTimer += dt;
      if (adcsTimer > 0.05f)
      {
        IMU::Reading imuReading = sat.imu.sample(*sat.body, gravity, adcsTimer);
        Magnetometer::Reading magReading = sat.magnetometer.sample(*sat.body, fieldNow, adcsTimer);
        glm::vec3 sunDirWorld = adcs.sunPosition - sat.body->position;
        StarTracker::Reading starReading = sat.starTracker.sample(*sat.body, sunDirWorld);
        SunSensor::Reading sunReading = sat.sunSensor.sample(*sat.body, sunDirWorld);

        FSWInputs inputs;
        inputs.imu = {imuReading.gyro, imuReading.accel};
        inputs.mag = {magReading.field, true};
        inputs.star = {starReading.attitude, starReading.valid};
        inputs.sunSensor = {sunReading.sunDirBody, sunReading.valid};
        // Last-known EPS telemetry -- this cycle's own consumption (wheels/
        // torquers, computed below from what step() is about to command)
        // hasn't happened yet, same "read before this cycle's effects"
        // relationship inputs.wheelTelemetry[i].speedRadS already has with
        // the wheel commands step() is about to issue.
        inputs.power = {sat.battery.stateOfCharge(), sat.battery.voltage()};
        for (int i = 0; i < NUM_WHEELS; i++)
          inputs.wheelTelemetry[i] = {sat.wheels[i]->currentSpeed, sat.wheels[i]->healthFactor > 0.01f};
        inputs.spacecraftPositionWorld = sat.body->position; // real orbital position (ECI meters) -- stands in for a real nav solution

        FSWOutputs out = adcs.step(inputs, adcsTimer);

        for (int i = 0; i < NUM_WHEELS; i++)
          sat.wheels[i]->commandTorque(out.wheelCommands[i].torqueNm);
        for (int i = 0; i < NUM_TORQUERS; i++)
          sat.magnetorquers[i]->commandDipoleMoment(out.torquerCommands[i].momentAm2);

        // =================== EPS (same 20 Hz cycle) ===================
        // Generation: sum every panel's cosine-law output against the same
        // sun direction the star tracker/sun sensor were just sampled
        // against -- zero while the real orbital position is in Earth's
        // shadow (see EclipseModel::inEclipse above), closing this
        // project's former "no orbital eclipse model" gap. Consumption: a
        // fixed housekeeping/sensor draw plus each actuator's
        // idle-plus-effort power for the commands just issued above -- see
        // the Config::POWER_* comments for the model each term follows.
        // Net power integrates straight into the battery.
        float genW = 0.0f;
        if (!inEclipse)
          for (const SolarPanel &panel : sat.solarPanels)
            genW += panel.sample(*sat.body, sunDirWorld, Config::SOLAR_FLUX_WM2).powerW;

        float drawW = Config::POWER_OBC_BASELINE_W + Config::POWER_IMU_W +
                      Config::POWER_MAGNETOMETER_W + Config::POWER_STAR_TRACKER_W +
                      Config::POWER_SUN_SENSOR_W;
        for (int i = 0; i < NUM_WHEELS; i++)
          drawW += Config::WHEEL_IDLE_POWER_W +
                   std::abs(out.wheelCommands[i].torqueNm * sat.wheels[i]->currentSpeed) / Config::WHEEL_MOTOR_EFFICIENCY;
        for (int i = 0; i < NUM_TORQUERS; i++)
          drawW += Config::TORQUER_IDLE_POWER_W +
                   std::abs(out.torquerCommands[i].momentAm2) * Config::TORQUER_POWER_PER_AM2_W;

        sat.battery.update(genW - drawW, adcsTimer);
        telemetry.netPowerW.push(genW - drawW);

        adcsTimer = 0.0f;

        // Ground-truth diagnostic -- computed here, not by ADCS (which has
        // no way to know body->orientation at all anymore), the same way
        // this project's earlier true-vs-estimated pointing error
        // comparisons always required direct simulation access.
        glm::quat trueErrQ = glm::inverse(sat.body->orientation) * adcs.targetAttitude;
        if (trueErrQ.w < 0.0f)
          trueErrQ = -trueErrQ;
        trueErrDeg = glm::degrees(2.0f * std::acos(glm::clamp(trueErrQ.w, -1.0f, 1.0f)));

        // Pushed once per ADCS cycle (a new sensor reading actually
        // exists), not once per render frame.
        telemetry.gyroMagDegS.push(glm::degrees(glm::length(adcs.lastGyroBody)));
        telemetry.accelMagMs2.push(glm::length(adcs.lastAccelBody));
        telemetry.magFieldMagUt.push(glm::length(adcs.magFieldBody) * 1e6f);
        telemetry.estimatedPointingErrorDeg.push(adcs.estimatedPointingErrorDeg);
        telemetry.truePointingErrorDeg.push(trueErrDeg);
        telemetry.batterySocPct.push(sat.battery.stateOfCharge() * 100.0f);
      }

      // =================== PHYSICS ===================
      world.step(dt);
    }

    // =================== DRAW ===================
    gui.beginFrame();
    imguiLayer.beginFrame();

    // =================== ORBIT VISUALIZATION ===================
    // Earth and the predicted orbit path, both in real ECI meters --
    // PhysicsWorld's own frame here *is* ECI (Earth's center at the world
    // origin, sat.body->position bridged from orbitState every frame
    // below), so the satellite's own wireframe/wheels/rods/etc. (drawn
    // further down, already reading sat.body->position/orientation) are
    // already at the right place -- no separate marker needed.
    drawEarth(gui, earthTexture, currentJdNow);
    drawOrbitPath(gui, orbitPathPoints);
    drawGroundFootprint(gui, orbitState.position, glm::radians(Config::FOOTPRINT_MIN_ELEVATION_DEG));

    drawSatelliteWireframe(gui, sat.body);
    drawReactionWheels(gui, sat.wheels, sat.body);
    drawMagnetorquers(gui, sat.magnetorquers, sat.body);
    drawStarTracker(gui, sat.starTracker, sat.body, adcs);
    drawMagneticField(gui, fieldNow, sat.body->position);
    drawMirror(gui, sat.body);
    drawSunReflection(gui, sat.body, adcs.sunPosition);

    gui.drawSphere(adcs.target, Config::TARGET_MARKER_RADIUS_M, {0, 1.0f, 0});

    // Sun marker sized to its *real* angular diameter (~32 arcmin) at its
    // current (arbitrary, scaled-for-visibility) distance from the
    // satellite, rather than a fixed prop radius -- r = d*tan(halfAngle).
    float sunDistance = glm::length(adcs.sunPosition - sat.body->position);
    float sunRadius = sunDistance * std::tan(glm::radians(Config::SUN_ANGULAR_DIAMETER_DEG * 0.5f));
    gui.drawSphere(adcs.sunPosition, sunRadius, {1.0f, 0.9f, 0.1f});

    // Pointing-error visualization: a line from the body straight to each
    // reference makes the *angular gap* between where the body actually
    // points and where it should legible at a glance -- much easier to
    // judge by eye than comparing the wireframe's own +Z arrow (drawn in
    // drawSatelliteWireframe) against a distant marker.
    gui.drawLine(sat.body->position, adcs.target, {0, 1.0f, 0});
    gui.drawLine(sat.body->position, adcs.sunPosition, {1.0f, 0.9f, 0.1f});

    drawWorldAxesGizmo(gui);

    drawADCSPanel(adcs, sat, telemetry, sim, epoch, orbitState, earthTexture, groundTrackLatLonDeg, currentJdNow, trueErrDeg, inEclipse);

    imguiLayer.endFrame();
    gui.endFrame();
  }
  return 0;
}
