#include "Cubesat.h"
#include "Config.h"
#include <glm/gtc/constants.hpp>
#include <cmath>
#include <memory>

// Composite mass/inertia for the mirror+bus spacecraft, both coaxially
// centered on the same point (a deployable-membrane layout like IKAROS's
// real solar sail -- boom-mounted mirror, bus at the hub -- so no
// parallel-axis term is needed; both contributions are already about the
// shared center).
//
// Mirror: 18m x 18m x 5mm box. Areal density 0.1 kg/m^2 (~2x IKAROS's
// real ~48 g/m^2 sail-only figure, accounting for a mirror needing
// better optical/structural quality than a pure photon-pressure sail)
// gives ~32.4 kg raw; rounded up to 35 kg for supporting boom/structure
// margin -- representative, not a real hardware spec (none exists at
// this scale). Standard thin-plate box inertia (thickness term
// negligible against the 18m span).
//
// Bus: 0.5m cube core, 40 kg representative small-bus mass (avionics,
// mirror-deployment mechanism). The "~2.5m envelope with panels
// deployed" from the mission concept is the deployed solar-panel span,
// not the structural core -- those panels are thin/low-mass, and their
// inertia contribution is small enough to fold into the bus's own
// margin rather than model as a separate element. Standard cube inertia.
namespace
{
constexpr float MIRROR_SPAN_M = 18.0f;
constexpr float MIRROR_THICKNESS_M = 0.005f;
constexpr float MIRROR_AREAL_DENSITY_KGM2 = 0.1f;
constexpr float MIRROR_MASS_KG = 35.0f; // rounded up from areal-density estimate for structure margin
constexpr float BUS_CORE_SIDE_M = 0.5f;
constexpr float BUS_MASS_KG = 40.0f;
constexpr float TOTAL_MASS_KG = MIRROR_MASS_KG + BUS_MASS_KG;

glm::mat3 computeCompositeInertiaTensor()
{
  // Thin-plate box: Ix = Iy = m*(L^2 + t^2)/12 ~= m*L^2/12 (t << L),
  // Iz = m*(Lx^2 + Ly^2)/12 = m*L^2/6 for a square plate.
  float mirrorIxy = MIRROR_MASS_KG * (MIRROR_SPAN_M * MIRROR_SPAN_M + MIRROR_THICKNESS_M * MIRROR_THICKNESS_M) / 12.0f;
  float mirrorIz = MIRROR_MASS_KG * (MIRROR_SPAN_M * MIRROR_SPAN_M + MIRROR_SPAN_M * MIRROR_SPAN_M) / 12.0f;

  // Cube: I = m*s^2/6, all axes.
  float busI = BUS_MASS_KG * (BUS_CORE_SIDE_M * BUS_CORE_SIDE_M) / 6.0f;

  return glm::mat3(
      mirrorIxy + busI, 0.0f, 0.0f,
      0.0f, mirrorIxy + busI, 0.0f,
      0.0f, 0.0f, mirrorIz + busI);
}
} // namespace

Cubesat buildCubesatPyramid(PhysicsWorld &world, HardwareConfig &outHw)
{
  Cubesat sat;
  sat.body = world.createBody(
      RigidBodyShape::BOX,
      glm::vec3(MIRROR_SPAN_M, MIRROR_SPAN_M, MIRROR_THICKNESS_M),
      MIRROR_MASS_KG);

  // Override the shape-derived (mirror-plate-only) mass/inertia with the
  // true composite mirror+bus values computed above -- see
  // RigidBody::setInertiaTensor()'s own header comment: this is exactly
  // its intended use (a scenario with a more accurate measured/derived
  // inertia than the single-shape primitive can compute).
  sat.body->setMass(TOTAL_MASS_KG);
  sat.body->setInertiaTensor(computeCompositeInertiaTensor());

  // Real orbital position (overwritten every frame from orbitState, in
  // ECI meters -- see main()) means this body's Z genuinely crosses zero
  // twice per orbit around an Earth centered at the world origin. The
  // engine's ground-collision resolution assumes a literal Z=0 ground
  // plane, which doesn't exist here, so this body opts out of it.
  sat.body->groundCollisionEnabled = false;

  // IMU board mounted in a corner of the bus, not at the center of mass --
  // like a real PCB, so its accelerometer isn't trivially always-zero (it
  // picks up centripetal/tangential terms from body rotation).
  sat.imu = IMU(glm::vec3(0.15f, 0.15f, 0.1f));

  // Pyramid layout: 4 wheels, each spin axis tilted `skew` from body +Z,
  // spaced 90 degrees apart in azimuth. Mounted in a small cluster near the
  // +Z face rather than at the body center, matching how a real RWA pyramid
  // bracket is bolted to one panel. Mount offsets scaled up from the old
  // 1U-cubesat's mm-scale mounts to this spacecraft's 0.5m bus core.
  const float skew = glm::radians(45.0f);
  const float mountRadius = 0.15f;
  const float mountHeight = 0.2f;

  for (int i = 0; i < 4; ++i)
  {
    float azimuth = glm::radians(45.0f) + i * glm::half_pi<float>(); // 45, 135, 225, 315 deg

    glm::vec3 axis(std::sin(skew) * std::cos(azimuth),
                   std::sin(skew) * std::sin(azimuth),
                   std::cos(skew));

    glm::vec3 mountPos(mountRadius * std::cos(azimuth),
                       mountRadius * std::sin(azimuth),
                       mountHeight);

    // Real smallsat-class reaction wheel sizing (~1 Nms momentum, ~20-30
    // mNm torque -- e.g. Blue Canyon RWp500 / Sinclair RW-1.0 class),
    // sized to the *bus's* mass/power/volume budget (75kg, see
    // computeCompositeInertiaTensor() above) rather than the mirror
    // payload's own huge inertia -- a real smallsat buys an off-the-shelf
    // wheel that fits its bus, it doesn't size a custom wheel to the
    // payload. wheelInertia is back-derived to keep momentum capacity
    // consistent with that class at maxSpeedRadS
    // (1.0 Nms / 628 rad/s ~= 1.6e-3 kg*m^2). At this modest torque
    // against the mirror's ~950-1900 kg*m^2 inertia, ADCS's own
    // torque-aware autoTune() (see ADCS.cpp/Controllers.cpp) is what
    // keeps the control loop from demanding more than this can deliver
    // -- settling is correspondingly slow, not a bug.
    const float maxTorqueNm = 0.025f;
    const float maxSpeedRadS = 6000.0f * (2.0f * glm::pi<float>() / 60.0f); // 6000 RPM max -- motor speed doesn't scale with spacecraft size
    const float wheelInertia = 1.6e-3f;                                     // kg*m^2 -- ~1.0 Nms momentum capacity at maxSpeedRadS

    auto wheel = std::make_unique<ReactionWheel>(mountPos, axis, maxTorqueNm, maxSpeedRadS, wheelInertia);

    sat.wheels.push_back(wheel.get());
    outHw.wheels[i] = {axis, maxTorqueNm, maxSpeedRadS, wheelInertia};
    sat.body->addForceGenerator(std::move(wheel));
  }

  // Magnetorquer cluster: 3 mutually orthogonal rods along the body axes,
  // the standard cubesat layout (unlike the wheels' skewed pyramid, there's
  // no benefit to tilting a torque rod -- it has no momentum to distribute
  // across axes, so straight body-axis alignment gives the cleanest
  // allocation). Mounts scaled up to the 0.5m bus core; 15 A*m^2 matches a
  // real torque-rod product in this bus class (e.g. ZARM Technik MT15-1).
  const glm::vec3 torquerAxes[3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  const glm::vec3 torquerMounts[3] = {{0.2f, 0, 0}, {0, 0.2f, 0}, {0, 0, -0.2f}};
  const float maxMomentAm2 = 15.0f;
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
  sat.magnetometer = Magnetometer(glm::vec3(-0.15f, -0.15f, 0.1f));

  // Star tracker boresight along body -Z (StarTracker's own default) --
  // opposite the +Z payload/pointing axis every guidance mode here aims,
  // so it isn't staring straight at whatever TARGET/SUN_POINTING/NADIR is
  // currently pointing +Z toward. Real placement follows the same logic:
  // keep the tracker away from the sun-facing/payload side.

  // Solar panels: one body-mounted cell array per face, the standard
  // cubesat layout (vs. a single sun-tracking array) -- whichever face(s)
  // happen to be sunward generate, the rest don't, so generation is a
  // direct function of attitude rather than something a gimbal hides.
  // panelAreaM2 is a representative deployed-panel dimension (not
  // sat.body->size, which is now the 18m mirror, not the panel envelope):
  // ~1.0m x 1.0m per face, consistent with the mission concept's "~2.5m
  // envelope with panels deployed" bus. ~28% is a representative
  // conversion efficiency for a triple-junction cell (vs. ~20% for
  // cheaper silicon).
  const float panelAreaM2 = 1.0f; // one representative 1.0m x 1.0m deployed panel face
  const float panelEfficiency = 0.28f;
  const glm::vec3 panelNormals[6] = {
      {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
  for (const glm::vec3 &n : panelNormals)
    sat.solarPanels.emplace_back(n, panelAreaM2, panelEfficiency);

  // Battery: a representative Li-ion pack sized up for this bus (40 Wh,
  // vs. the old 1U-cubesat's 10 Wh) -- bigger bus means more housekeeping
  // and actuator draw (see the new wheel/torquer sizing above), while
  // still being small enough that a deliberately harsh test (Simulation
  // tab's "Drain Battery" button, eclipse, or a long run with poor
  // sun-facing geometry) can bring it down to FDIR's low-battery threshold.
  sat.battery = Battery(40.0f /* Wh */, 6.0f /* V empty */, 8.4f /* V full */, 0.8f /* initial SOC */);

  outHw.busInertiaTensor = sat.body->inertiaTensor;

  return sat;
}

FSWInputs Cubesat::sampleSensors(float dt)
{
  FSWInputs in;

  IMU::Reading imuR = imu.sample(*body, gravity, dt);
  in.imu = {imuR.gyro, imuR.accel};

  Magnetometer::Reading magR = magnetometer.sample(*body, ambientFieldWorld, dt);
  in.mag = {magR.field, true};

  StarTracker::Reading starR = starTracker.sample(*body, sunDirWorld);
  in.star = {starR.attitude, starR.valid};

  SunSensor::Reading sunR = sunSensor.sample(*body, sunDirWorld);
  // SunSensor has no eclipse model of its own (see its header: "valid is
  // always true here... a scenario that wants eclipse-aware dropouts needs
  // to gate this externally") -- a real coarse sun sensor reports no lock
  // when the sun itself is physically blocked by Earth, not just when the
  // geometric direction happens to be undefined, so that blindness is
  // applied here. This also keeps TRIAD fallback (computeTriadFallback,
  // ADCS.cpp) honest during eclipse: it already requires in.sunSensor.valid,
  // but without this gate it would happily TRIAD off a sun direction the
  // satellite couldn't actually observe.
  in.sunSensor = {sunR.sunDirBody, sunR.valid && !inEclipse};

  in.power = {battery.stateOfCharge(), battery.voltage()};

  for (int i = 0; i < NUM_WHEELS; i++)
    in.wheelTelemetry[i] = {wheels[i]->currentSpeed, wheels[i]->healthFactor > 0.01f};

  in.spacecraftPositionWorld = body->position; // real orbital position (ECI meters) -- stands in for a real nav solution

  return in;
}

void Cubesat::applyActuatorCommands(const FSWOutputs &out)
{
  for (int i = 0; i < NUM_WHEELS; i++)
    wheels[i]->commandTorque(out.wheelCommands[i].torqueNm);
  for (int i = 0; i < NUM_TORQUERS; i++)
    magnetorquers[i]->commandDipoleMoment(out.torquerCommands[i].momentAm2);
}

float Cubesat::updatePower(float dt, const FSWOutputs &out)
{
  float genW = 0.0f;
  if (!inEclipse)
    for (const SolarPanel &panel : solarPanels)
      genW += panel.sample(*body, sunDirWorld, Config::SOLAR_FLUX_WM2).powerW;

  float drawW = Config::POWER_OBC_BASELINE_W + Config::POWER_IMU_W +
                Config::POWER_MAGNETOMETER_W + Config::POWER_STAR_TRACKER_W +
                Config::POWER_SUN_SENSOR_W;
  for (int i = 0; i < NUM_WHEELS; i++)
    drawW += Config::WHEEL_IDLE_POWER_W +
             std::abs(out.wheelCommands[i].torqueNm * wheels[i]->currentSpeed) / Config::WHEEL_MOTOR_EFFICIENCY;
  for (int i = 0; i < NUM_TORQUERS; i++)
    drawW += Config::TORQUER_IDLE_POWER_W +
             std::abs(out.torquerCommands[i].momentAm2) * Config::TORQUER_POWER_PER_AM2_W;

  float netW = genW - drawW;
  battery.update(netW, dt);
  return netW;
}
