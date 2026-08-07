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
//   [1-7] pointing mode: Nadir / Sun / Detumble / Target / Slew / Fine / Reflect
//   [T]   kick the body into a random tumble (to test Detumble)
//
// Target/Slew/Fine/Reflect modes aim at adcs.target, which auto-tracks
// the closest ground station (GroundStations.h) currently meeting the
// footprint's minimum-elevation requirement -- no manual target selection
// needed; wheel faults are manual-only, via the Simulation tab.
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
Cubesat buildCubesatPyramid(PhysicsWorld &world, HardwareConfig &outHw);
