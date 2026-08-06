#include <vgl/vgl.h>
#include <rigidbody/PhysicsWorld.h>
#include <rigidbody/actuators/ReactionWheel.h>
#include <rigidbody/actuators/Magnetorquer.h>
#include <rigidbody/sensors/IMU.h>
#include <rigidbody/sensors/Magnetometer.h>
#include <rigidbody/sensors/StarTracker.h>
#include <rigidbody/sensors/SunSensor.h>
#include <rigidbody/environment/MagneticField.h>
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

  sat.body->position.z = sat.body->size.y * 3; // float above the ground

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
// Random points on the unit sphere, scaled out -- one for arbitrary
// pointing targets, one (farther out, for visual distinction) for the sun.
// ---------------------------------------------------------------------------
static glm::vec3 randomTarget()
{
  static std::mt19937 rng(std::random_device{}());
  static std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
  glm::vec3 v(dist(rng), dist(rng), dist(rng));
  return glm::normalize(v);
}

static glm::vec3 randomSunPosition()
{
  static std::mt19937 rng(std::random_device{}());
  static std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
  glm::vec3 v(dist(rng), dist(rng), dist(rng));
  return glm::normalize(v) * 2.0f;
}

// ---------------------------------------------------------------------------
// Randomly fails or degrades a wheel every so often, independent of
// anything the flight software does -- a real bearing/driver fault doesn't
// wait for a convenient moment. Faults persist once triggered; either
// restart the scenario or use repairAll() to clear them.
// ---------------------------------------------------------------------------
struct WheelFaultInjector
{
  std::mt19937 rng{std::random_device{}()};
  std::exponential_distribution<float> nextFaultDist;
  float timeUntilNextFault;
  float meanSecondsBetweenFaults;

  explicit WheelFaultInjector(float meanSecondsBetweenFaultsIn)
      : nextFaultDist(1.0f / meanSecondsBetweenFaultsIn),
        meanSecondsBetweenFaults(meanSecondsBetweenFaultsIn)
  {
    timeUntilNextFault = nextFaultDist(rng);
  }

  // Rebuilds the exponential distribution around a new mean -- for a
  // Simulation-panel slider to retune the fault rate live, without
  // restarting the scenario.
  void setMeanSecondsBetweenFaults(float mean)
  {
    mean = std::max(mean, 1.0f);
    if (std::abs(mean - meanSecondsBetweenFaults) < 1e-6f)
      return;
    meanSecondsBetweenFaults = mean;
    nextFaultDist = std::exponential_distribution<float>(1.0f / mean);
  }

  void update(float dt, std::vector<ReactionWheel *> &wheels)
  {
    timeUntilNextFault -= dt;
    if (timeUntilNextFault > 0.0f)
      return;
    trigger(wheels);
  }

  void trigger(std::vector<ReactionWheel *> &wheels)
  {
    timeUntilNextFault = nextFaultDist(rng);

    std::uniform_int_distribution<int> pickWheel(0, (int)wheels.size() - 1);
    ReactionWheel *w = wheels[pickWheel(rng)];

    std::uniform_real_distribution<float> roll(0.0f, 1.0f);
    if (roll(rng) < 0.4f)
    {
      w->healthFactor = 0.0f; // complete failure
    }
    else
    {
      std::uniform_real_distribution<float> degrade(0.1f, 0.6f);
      w->healthFactor = degrade(rng); // reduced torque, not dead
    }
  }

  void repairAll(std::vector<ReactionWheel *> &wheels)
  {
    for (auto *w : wheels)
      w->healthFactor = 1.0f;
  }
};

// ---------------------------------------------------------------------------
// Config
// ---------------------------------------------------------------------------
namespace Config
{
  // Camera
  constexpr float CAMERA_NEAR = 0.1f;
  constexpr float CAMERA_FAR = 100.0f;
  constexpr float CAMERA_FOV = 45.0f;
  constexpr float CAMERA_INITIAL_DISTANCE = 1.0f;
  constexpr float CAMERA_MIN_DISTANCE = 0.1f;
  constexpr float CAMERA_MAX_DISTANCE = 100.0f;
  constexpr float ZOOM_SENSITIVITY = 1.0f;
  constexpr float PAN_SENSITIVITY = 0.2f;

  // Coordinate grid: three walls of a box (floor + two back walls) meeting
  // at a corner behind/below the satellite, acting as a visual coordinate
  // reference rather than an infinite ground plane -- sized against the
  // 10cm cubesat itself (half-size a few times its 0.25m body-axis arrows),
  // not the old 5m/1m ground-plane scale that dwarfed it.
  constexpr float GRID_HALF_SIZE = 3.0f;
  constexpr float GRID_STEP = 0.5f;

  // Plot panel (world space, Y=-1.5 plane, X in [-0.5, 0.5], Z up)
  const glm::vec3 PLOT_ORIGIN{-0.5f, -1.5f, 0.0f};
  constexpr float PLOT_WIDTH = 1.0f;
  constexpr float PLOT_HEIGHT = 0.18f;
  constexpr float PLOT_GAP = 0.05f;

  constexpr float MEAN_SECONDS_BETWEEN_FAULTS = 25.0f;
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

  // EPS (electrical power subsystem) power budget: representative
  // wattages for every load on the bus, summed each ADCS cycle against
  // what the solar panels generate that same cycle -- see the FLIGHT
  // SOFTWARE block in main(). No orbital eclipse model exists in this sim,
  // so there's no shadow term; the sun is always "up" from wherever it's
  // currently placed.
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

// Draws one axis-aligned grid plane: a fine lattice of lines over the two
// "free" axes, held fixed at `fixedOffset` along the third (`fixedAxis`,
// 0=X/1=Y/2=Z), spanning [-halfSize, halfSize] around `center` in the free
// axes. The line running through each free axis's zero -- i.e. the pair
// that crosses directly behind/below `center` -- is drawn in that axis's
// own color instead of the faint grid color, so the two axis-color lines
// on every wall read as "this plane's local coordinate cross," the same
// role the colored line-through-the-origin played in the old single-plane
// ground grid.
static void drawGridPlane(GUI &gui, const glm::vec3 &center, int fixedAxis, float fixedOffset,
                          float halfSize, float step, const glm::vec3 &gridColor,
                          const glm::vec3 &axisColorU, const glm::vec3 &axisColorV)
{
  int u = (fixedAxis + 1) % 3;
  int v = (fixedAxis + 2) % 3;

  for (float i = -halfSize; i <= halfSize + 1e-4f; i += step)
  {
    bool onAxis = std::abs(i) < 1e-4f;

    glm::vec3 p0(0.0f), p1(0.0f);
    p0[fixedAxis] = p1[fixedAxis] = fixedOffset;
    p0[u] = p1[u] = i;
    p0[v] = -halfSize;
    p1[v] = halfSize;
    gui.drawLine(center + p0, center + p1, onAxis ? axisColorU : gridColor);

    glm::vec3 q0(0.0f), q1(0.0f);
    q0[fixedAxis] = q1[fixedAxis] = fixedOffset;
    q0[v] = q1[v] = i;
    q0[u] = -halfSize;
    q1[u] = halfSize;
    gui.drawLine(center + q0, center + q1, onAxis ? axisColorV : gridColor);
  }
}

// Three walls of a box -- floor, back wall, side wall -- meeting at the
// corner behind/below `center`, standing in for a coordinate grid at this
// scenario's own (tiny, 10cm-cubesat) scale rather than the far larger
// ground-plane grid most other scenarios want. Colors match the body-axis
// arrows drawSatelliteWireframe() draws (X red, Y green, Z blue) so the
// grid reads as an extension of the satellite's own body frame, not an
// unrelated backdrop.
static void drawSatelliteCoordinateBox(GUI &gui, const glm::vec3 &center, float halfSize, float step)
{
  const glm::vec3 gridColor{0.16f, 0.19f, 0.26f};
  const glm::vec3 axisColorX{0.75f, 0.3f, 0.32f};
  const glm::vec3 axisColorY{0.32f, 0.68f, 0.4f};
  const glm::vec3 axisColorZ{0.32f, 0.48f, 0.85f};

  // Floor (Z fixed, below center)
  drawGridPlane(gui, center, 2, -halfSize, halfSize, step, gridColor, axisColorX, axisColorY);
  // Back wall (Y fixed, behind center)
  drawGridPlane(gui, center, 1, -halfSize, halfSize, step, gridColor, axisColorY, axisColorZ);
  // Side wall (X fixed, behind center)
  drawGridPlane(gui, center, 0, -halfSize, halfSize, step, gridColor, axisColorZ, axisColorX);
}

// Body drawn as a 12-edge wireframe box instead of a solid box.
void drawSatelliteWireframe(GUI &gui, RigidBody *sat)
{
  glm::vec3 h = sat->size * 0.5f;
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

  float arrowLength = 0.25f;
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
  const float wheelRadius = 0.02f;
  const float wheelThickness = 0.006f;
  const float arrowLength = 0.05f;

  glm::vec3 totalAngular{0};
  for (auto &wheel : reactionWheels)
  {
    glm::vec3 worldPos = wheel->getWorldMountPosition(*sat);
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
  const float rodRadius = 0.006f;
  const float rodLength = 0.07f;
  const float arrowLength = 0.06f;

  for (auto &rod : magnetorquers)
  {
    glm::vec3 worldPos = rod->getWorldMountPosition(*sat);
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
  const float length = 0.5f;
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
  glm::vec3 mountOffsetBody = Config::MIRROR_NORMAL_BODY * (sat->size.z * 0.5f + Config::MIRROR_SIZE.z * 0.5f);
  glm::vec3 worldPos = sat->position + sat->orientation * mountOffsetBody;
  gui.drawBox(worldPos, Config::MIRROR_SIZE, sat->orientation, {0.85f, 0.92f, 0.98f});
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
  glm::vec3 mountOffsetBody = Config::MIRROR_NORMAL_BODY * (sat->size.z * 0.5f + Config::MIRROR_SIZE.z * 0.5f);
  glm::vec3 mirrorPos = sat->position + sat->orientation * mountOffsetBody;
  glm::vec3 normalWorld = glm::normalize(sat->orientation * Config::MIRROR_NORMAL_BODY);

  glm::vec3 incidentDir = glm::normalize(mirrorPos - sunPosition); // sun -> mirror
  gui.drawLine(sunPosition, mirrorPos, {1.0f, 0.5f, 0.9f});

  // Front face is illuminated only if the normal points back toward the
  // sun relative to the incoming ray, i.e. dot(normal, -incident) > 0.
  if (glm::dot(normalWorld, -incidentDir) <= 0.0f)
    return;

  glm::vec3 reflectedDir = incidentDir - 2.0f * glm::dot(incidentDir, normalWorld) * normalWorld;
  gui.drawLine(mirrorPos, mirrorPos + reflectedDir * Config::REFLECTED_RAY_LENGTH, {1.0f, 0.5f, 0.9f});
}

// Visualizes the ambient field the satellite is sitting in: a small grid of
// arrows around the body, all pointing the same direction/magnitude, since
// the field varies negligibly over a cubesat-sized volume (its gradient
// scale is Earth-sized, not cubesat-sized) -- what varies is the field
// *over time* as the kinematic orbit in MagneticField sweeps through it
// (see MagneticField.h). A distinct, brighter arrow at the body itself
// marks the same vector so it reads clearly against the grid.
static void drawMagneticField(GUI &gui, const glm::vec3 &fieldWorldT, glm::vec3 satPos)
{
  // Scales a ~20-60 uT LEO field into a visible arrow length at cubesat
  // scene scale (satellite ~0.1m, grid spacing 1m).
  constexpr float FIELD_VISUAL_SCALE = 8000.0f;
  const glm::vec3 fieldColor{0.2f, 0.9f, 0.9f};

  glm::vec3 arrow = fieldWorldT * FIELD_VISUAL_SCALE;

  gui.drawArrow(satPos, satPos + arrow, fieldColor, 2.0f);

  const int gridN = 2; // -gridN..+gridN in each of x,y -> 5x5 grid
  const float spacing = 0.6f;
  const float gridHeight = -0.7f; // below the satellite, out of the way of the sat/wheels/rods
  for (int ix = -gridN; ix <= gridN; ++ix)
  {
    for (int iy = -gridN; iy <= gridN; ++iy)
    {
      glm::vec3 base = satPos + glm::vec3(ix * spacing, iy * spacing, gridHeight);
      gui.drawArrow(base, base + arrow, fieldColor * 0.6f);
    }
  }
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
static void drawEpsTab(Cubesat &sat, ADCS &adcs, SensorTelemetry &telemetry)
{
  float soc = sat.battery.stateOfCharge();
  ImVec4 socColor = soc > 0.5f ? ImVec4(0.3f, 1.0f, 0.4f, 1.0f)
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
    totalGenW += r.powerW;
    const char *name = i < 6 ? panelNames[i] : "?";
    ImGui::Text("%s face: %5.2f W  (incidence %5.1f deg)", name, r.powerW, r.incidenceAngleDeg);
  }
  ImGui::Text("Total generation: %.2f W", totalGenW);
  ImGui::TextDisabled("No orbital eclipse model -- the sun is always \"up,\" generation only depends on attitude.");
}

// Simulation-level knobs, as opposed to ADCS/FSW state -- things that
// belong to the test harness around the spacecraft, not to the spacecraft
// itself. Held by value in main() and handed to drawSimulationTab() by
// reference each frame, same pattern as ADCS's own public fields.
struct SimControls
{
  bool paused = false;
  bool faultInjectionEnabled = true;
  float tumbleKickRadS;

  explicit SimControls(float tumbleKickRadSIn) : tumbleKickRadS(tumbleKickRadSIn) {}
};

static void drawSimulationTab(SimControls &sim, WheelFaultInjector &faultInjector,
                              std::vector<ReactionWheel *> &wheels, RigidBody *body, ADCS &adcs)
{
  ImGui::SeparatorText("Simulation");
  ImGui::Checkbox("Pause simulation", &sim.paused);
  ImGui::TextDisabled("Physics, FSW, and fault injection all freeze; camera/UI stay live.");

  ImGui::SeparatorText("Disturbances");
  ImGui::DragFloat("Tumble kick (rad/s)", &sim.tumbleKickRadS, 0.05f, 0.0f, 5.0f, "%.2f");
  if (ImGui::Button("Kick into random tumble [T]"))
  {
    static std::mt19937 tumbleRng(std::random_device{}());
    std::uniform_real_distribution<float> d(-sim.tumbleKickRadS, sim.tumbleKickRadS);
    body->angularVelocity = glm::vec3(d(tumbleRng), d(tumbleRng), d(tumbleRng));
  }

  ImGui::SeparatorText("Reaction Wheel Faults");
  ImGui::Checkbox("Enable random fault injection", &sim.faultInjectionEnabled);
  float mean = faultInjector.meanSecondsBetweenFaults;
  if (ImGui::DragFloat("Mean seconds between faults", &mean, 1.0f, 1.0f, 300.0f, "%.0f"))
    faultInjector.setMeanSecondsBetweenFaults(mean);
  if (ImGui::Button("Trigger fault now [F]"))
    faultInjector.trigger(wheels);
  ImGui::SameLine();
  if (ImGui::Button("Repair all wheels"))
    faultInjector.repairAll(wheels);

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
// aren't part of the spacecraft itself: pause, induced tumbles, and the
// reaction wheel fault injector).
static void drawADCSPanel(ADCS &adcs, Cubesat &sat,
                          SensorTelemetry &telemetry, SimControls &sim,
                          WheelFaultInjector &faultInjector,
                          float trueErrDeg)
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
      drawEpsTab(sat, adcs, telemetry);
      ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Simulation"))
    {
      drawSimulationTab(sim, faultInjector, sat.wheels, sat.body, adcs);
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
                "CubeSat ADCS (Pyramid RWA)  |  [1-6] Mode  [Space] Target  [T] Tumble  [F] Fault  |  "
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
  adcs.sunPosition = randomSunPosition();
  float adcsTimer = 0.0f;
  float missionTime = 0.0f;
  float trueErrDeg = 0.0f; // harness-side diagnostic: true (ground-truth) pointing error, held between ADCS cycles like fieldNow below

  // No Gravity generator is attached to this world (free-floating orbit),
  // so ambient gravity for the IMU model is zero -- this is harness-side
  // simulation setup now, not something ADCS itself ever sees or assumes.
  glm::vec3 gravity{0.0f};

  // Ambient magnetic field model (see rigidbody/environment/MagneticField.h)
  // -- default LEO/ISS-like altitude and inclination. Sampled every frame
  // below and written into both the magnetorquers (which need it to turn a
  // commanded dipole moment into torque) and ADCS's ambientFieldWorld
  // (its TRIAD reference vector, the same category of "known model input"
  // as target/sunPosition -- see ADCS.h), the same way `gravity` feeds the
  // IMU sample below.
  MagneticField magneticField;

  SensorTelemetry telemetry(Config::TELEMETRY_HISTORY_SAMPLES);

  WheelFaultInjector faultInjector(Config::MEAN_SECONDS_BETWEEN_FAULTS);
  SimControls sim(Config::TUMBLE_KICK_RAD_S);
  glm::vec3 fieldNow{0.0f}; // last-sampled ambient field; held while paused rather than resampled

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

    if (gui.isKeyJustPressed(GLFW_KEY_F))
      faultInjector.trigger(sat.wheels);

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

      // Ambient field at the current simulated time -- fed to the
      // magnetorquers (they need it every physics substep to turn a
      // commanded dipole moment into torque) and to ADCS (it needs it to
      // interpret the magnetometer), same role adcs.gravity plays for the
      // IMU.
      fieldNow = magneticField.sample(missionTime);
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
        inputs.spacecraftPositionWorld = sat.body->position; // stands in for a real nav solution (no GPS/ephemeris modeled)

        FSWOutputs out = adcs.step(inputs, adcsTimer);

        for (int i = 0; i < NUM_WHEELS; i++)
          sat.wheels[i]->commandTorque(out.wheelCommands[i].torqueNm);
        for (int i = 0; i < NUM_TORQUERS; i++)
          sat.magnetorquers[i]->commandDipoleMoment(out.torquerCommands[i].momentAm2);

        // =================== EPS (same 20 Hz cycle) ===================
        // Generation: sum every panel's cosine-law output against the same
        // sun direction the star tracker/sun sensor were just sampled
        // against. Consumption: a fixed housekeeping/sensor draw plus each
        // actuator's idle-plus-effort power for the commands just issued
        // above -- see the Config::POWER_* comments for the model each
        // term follows. Net power integrates straight into the battery.
        float genW = 0.0f;
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

      if (sim.faultInjectionEnabled)
        faultInjector.update(dt, sat.wheels);

      // =================== PHYSICS ===================
      world.step(dt);
    }

    // =================== DRAW ===================
    gui.beginFrame();
    imguiLayer.beginFrame();
    drawSatelliteCoordinateBox(gui, sat.body->position, Config::GRID_HALF_SIZE, Config::GRID_STEP);
    drawSatelliteWireframe(gui, sat.body);
    drawReactionWheels(gui, sat.wheels, sat.body);
    drawMagnetorquers(gui, sat.magnetorquers, sat.body);
    drawStarTracker(gui, sat.starTracker, sat.body, adcs);
    drawMagneticField(gui, fieldNow, sat.body->position);
    drawMirror(gui, sat.body);
    drawSunReflection(gui, sat.body, adcs.sunPosition);

    gui.drawSphere(adcs.target, 0.05f, {0, 1.0f, 0});

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

    drawADCSPanel(adcs, sat, telemetry, sim, faultInjector, trueErrDeg);

    imguiLayer.endFrame();
    gui.endFrame();
  }
  return 0;
}
