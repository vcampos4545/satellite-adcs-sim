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

  glm::vec3 gravity{0.0f};
  glm::vec3 ambientFieldWorld{0.0f};
  glm::vec3 sunDirWorld{0.0f};
  bool inEclipse = false;

  // The only place simulated hardware is translated to/from the plain
  // FlightTypes.h contract FlightSoftware::step() actually runs on -- see
  // FlightSoftware.h's own header comment. Every sensor is sampled once,
  // at the shared FSW rate `dt`; no per-sensor scheduling.
  FSWInputs sampleSensors(float dt);

  // Applies FlightSoftware::step()'s output to the simulated wheels/
  // magnetorquers.
  void applyActuatorCommands(const FSWOutputs &out);

  // EPS accounting: solar generation (cosine law, zero in eclipse) minus
  // housekeeping/sensor draw plus each actuator's idle-plus-effort power
  // for `out` -- see Config::POWER_* for the model each term follows.
  // Integrates net power into `battery` and returns it (net watts) for the
  // caller's own telemetry. Reads `sunDirWorld`/`inEclipse` from this
  // object's own fields (set by the harness alongside the others above).
  float updatePower(float dt, const FSWOutputs &out);
};

// Builds the simulated hardware AND the matching HardwareConfig ADCS::
// configure() needs, from the exact same geometry in one place -- keeping
// them in two separate functions risks the flight-software config quietly
// drifting out of sync with what the simulated actuators actually are.
Satellite buildSatellite(PhysicsWorld &world, HardwareConfig &outHw);
