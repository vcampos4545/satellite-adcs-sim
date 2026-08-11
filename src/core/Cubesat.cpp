#include "Cubesat.h"
#include <glm/gtc/constants.hpp>
#include <cmath>
#include <memory>

Cubesat buildCubesatPyramid(PhysicsWorld &world, HardwareConfig &outHw)
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
