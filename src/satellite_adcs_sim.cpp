#include <vgl/vgl.h>
#include <rigidbody/PhysicsWorld.h>
#include <rigidbody/actuators/ReactionWheel.h>
#include <rigidbody/actuators/Magnetorquer.h>
#include <rigidbody/sensors/IMU.h>
#include <rigidbody/sensors/Magnetometer.h>
#include <rigidbody/sensors/StarTracker.h>
#include <rigidbody/sensors/SunSensor.h>
#include <rigidbody/environment/MagneticField.h>
#include "ADCS.h"
#include "FlightTypes.h"
#include "common/World.h"
#include "common/ImGuiLayer.h"
#include "common/Telemetry.h"
#include <random>
#include <memory>
#include <cmath>
#include <cstdio>
#include <algorithm>
#include <glm/gtc/constants.hpp>

// ---------------------------------------------------------------------------
// Full cubesat ADCS: pointing modes (Nadir/Sun/Detumble/Target/Slew/Fine),
// an IMU and magnetometer as the only sensors the flight software ever
// reads (see ADCS.h/.cpp), a 4-wheel reaction wheel pyramid plus a 3-axis
// magnetorquer cluster as actuators, and randomly-occurring reaction wheel
// faults (reduced torque or complete failure). The magnetorquers can either
// sit idle (reaction-wheel-only detumble/pointing, the original behavior)
// or take over DETUMBLE via the classic B-dot law -- see the Controller
// panel's Detumble Actuator selector.
//
// Controls:
//   [1-6]   pointing mode: Nadir / Sun / Detumble / Target / Slew / Fine
//   [Space] new random target (for Target/Slew/Fine modes)
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

    const float maxTorqueNm = 0.001f;                                  // same as the 3-wheel cubesat
    const float maxSpeedRadS = 6000.0f * (2.0f * glm::pi<float>() / 60.0f); // 6000 RPM max
    const float wheelInertia = 1e-6f;                                  // kg*m^2

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

  // Grid
  constexpr float GRID_SIZE = 5.0f;
  constexpr float GRID_STEP = 1.0f;

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

  explicit SensorTelemetry(int samples)
      : gyroMagDegS(samples), accelMagMs2(samples), magFieldMagUt(samples),
        estimatedPointingErrorDeg(samples), truePointingErrorDeg(samples) {}
};

// ---------------------------------------------------------------------------
// Draw helpers
// ---------------------------------------------------------------------------

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
  gui.drawLine(sunPosition, mirrorPos, {1.0f, 0.9f, 0.1f});

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
  static const char *modeNames[] = {"Nadir", "Sun Pointing", "Detumble", "Target", "Slew", "Fine Pointing"};
  int modeIdx = static_cast<int>(adcs.mode);
  if (ImGui::Combo("Pointing mode", &modeIdx, modeNames, IM_ARRAYSIZE(modeNames)))
    adcs.mode = static_cast<PointingMode>(modeIdx);

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
static void drawADCSPanel(ADCS &adcs, std::vector<ReactionWheel *> &wheels,
                          const std::vector<Magnetorquer *> &magnetorquers,
                          SensorTelemetry &telemetry, SimControls &sim,
                          WheelFaultInjector &faultInjector, RigidBody *body,
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
      drawActuatorsTab(adcs, wheels, magnetorquers);
      ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Simulation"))
    {
      drawSimulationTab(sim, faultInjector, wheels, body, adcs);
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
  World scene(WorldType::SPACE);
  scene.apply(gui);
  gui.camera
      .setUp({0, 0, 1})
      .setClipPlanes(Config::CAMERA_NEAR, Config::CAMERA_FAR)
      .setFOV(Config::CAMERA_FOV);

  OrbitalCamera orbit(
      Config::CAMERA_INITIAL_DISTANCE,
      45.0f, 0.0f,
      {0, 0, 0.3});
  orbit.setMaxDistance(Config::CAMERA_MAX_DISTANCE)
      .setMinDistance(Config::CAMERA_MIN_DISTANCE)
      .setZoomSensitivity(Config::ZOOM_SENSITIVITY)
      .setPanSensitivity(Config::PAN_SENSITIVITY);

  glm::vec2 lastMousePos = gui.getMousePosition();

  PhysicsWorld world;
  HardwareConfig hwConfig;
  Cubesat sat = buildCubesatPyramid(world, hwConfig);

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
        for (int i = 0; i < NUM_WHEELS; i++)
          inputs.wheelTelemetry[i] = {sat.wheels[i]->currentSpeed, sat.wheels[i]->healthFactor > 0.01f};
        inputs.spacecraftPositionWorld = sat.body->position; // stands in for a real nav solution (no GPS/ephemeris modeled)

        FSWOutputs out = adcs.step(inputs, adcsTimer);

        for (int i = 0; i < NUM_WHEELS; i++)
          sat.wheels[i]->commandTorque(out.wheelCommands[i].torqueNm);
        for (int i = 0; i < NUM_TORQUERS; i++)
          sat.magnetorquers[i]->commandDipoleMoment(out.torquerCommands[i].momentAm2);

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
      }

      if (sim.faultInjectionEnabled)
        faultInjector.update(dt, sat.wheels);

      // =================== PHYSICS ===================
      world.step(dt);
    }

    // =================== DRAW ===================
    gui.beginFrame();
    imguiLayer.beginFrame();
    scene.draw(gui, Config::GRID_SIZE, Config::GRID_STEP);
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

    drawADCSPanel(adcs, sat.wheels, sat.magnetorquers, telemetry, sim, faultInjector, sat.body, trueErrDeg);

    imguiLayer.endFrame();
    gui.endFrame();
  }
  return 0;
}
