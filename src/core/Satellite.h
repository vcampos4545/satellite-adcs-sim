#pragma once
#include <rigidbody/PhysicsWorld.h>
#include <rigidbody/actuators/ReactionWheel.h>
#include <rigidbody/actuators/Magnetorquer.h>
#include <rigidbody/sensors/IMU.h>
#include <rigidbody/sensors/Magnetometer.h>
#include <rigidbody/sensors/StarTracker.h>
#include <rigidbody/sensors/SunSensor.h>
#include <rigidbody/power/SolarPanel.h>
#include <rigidbody/power/Battery.h>
#include "fsw/FlightTypes.h"
#include <vector>

struct Satellite
{
  RigidBody *body = nullptr;
  std::vector<ReactionWheel *> wheels;
  std::vector<Magnetorquer *> magnetorquers;
  IMU imu;
  Magnetometer magnetometer;
  StarTracker starTracker;
  SunSensor sunSensor;
  std::vector<SolarPanel> solarPanels;
  Battery battery;

  // The plain-data mirror of wheels/magnetorquers' geometry/limits ADCS::
  // configure() needs -- populated by buildSatellite() from the exact same
  // SatelliteConfig.h constants/loop that build the real actuators above,
  // so this can never quietly drift out of sync with what's actually
  // simulated. Lives on Satellite itself (not threaded through separately)
  // since it's inherently a property of this specific spacecraft.
  HardwareConfig hwConfig;

  glm::vec3 gravity{0.0f};
  glm::vec3 ambientFieldWorld{0.0f};
  glm::vec3 sunDirWorld{0.0f};
  bool inEclipse = false;

  // No sampleSensors()/applyActuatorCommands() here -- FlightSoftware
  // samples imu/magnetometer/starTracker/sunSensor/wheels/battery
  // directly, each at its own realistic rate, and commands wheels/
  // magnetorquers directly too (see FlightSoftware.h's own header
  // comment). Keeping that independent of this struct is the point: this
  // sim's hardware objects are just hardware, with no bundled "sample
  // everything at once" concept of their own, the same way real sensor/
  // actuator drivers don't know or care what rate flight software polls
  // them at.

  // EPS accounting: solar generation (cosine law, zero in eclipse) minus
  // housekeeping/sensor draw plus each actuator's idle-plus-effort power,
  // read directly off wheels[i]->commandedTorque/magnetorquers[i]->
  // commandedDipoleMoment -- already set by FlightSoftware's own
  // commandTorque()/commandDipoleMoment() calls this cycle, so this needs
  // no actuator-command parameter of its own. See SatelliteConfig.h's
  // POWER_*/WHEEL_*/TORQUER_* constants for the model each term follows.
  // Integrates net power into `battery` and returns it (net watts) for
  // the caller's own telemetry. Reads
  // `sunDirWorld`/`inEclipse` from this object's own fields (set by the
  // harness alongside the others above).
  float updatePower(float dt);
};

// Builds the simulated hardware and its hwConfig mirror from the exact
// same geometry in one place -- see Satellite::hwConfig's own comment for
// why that matters.
Satellite buildSatellite(PhysicsWorld &world);
