#pragma once
#include "fsw/FlightSoftwareHAL.h"
#include "Cubesat.h"

// Implements FlightSoftwareHAL against this project's simulated hardware --
// exactly the sensor-sampling/actuator-command code that used to live
// inline in main()'s FSW block, now behind the interface FlightSoftware
// pulls from/pushes to on its own schedule. This is the piece a HIL adapter
// would eventually replace with a class implementing the same interface
// against real ADC/I2C/SPI reads and driver writes -- FlightSoftware itself
// wouldn't change.
//
// Environment context that changes every outer render frame (gravity,
// ambient field, sun direction, eclipse) isn't a sensor reading -- it's
// pushed in via setEnvironment(), called once per frame before the tick
// loop, the same role adcs.ambientFieldWorld/adcs.sunPosition already play
// as harness-pushed state on ADCS itself.
class SimFlightSoftwareHAL : public FlightSoftwareHAL
{
public:
  explicit SimFlightSoftwareHAL(Cubesat &sat);

  void setEnvironment(const glm::vec3 &gravity, const glm::vec3 &ambientFieldWorld,
                       const glm::vec3 &sunDirWorld, bool inEclipse);

  ImuSample readImu(float dt) override;
  MagSample readMag(float dt) override;
  StarTrackerSample readStarTracker() override;
  SunSensorSample readSunSensor() override;
  PowerSample readPower() override;
  std::array<WheelTelemetry, NUM_WHEELS> readWheelTelemetry() override;
  glm::vec3 readNavPosition() override;

  void commandWheelTorque(int index, float torqueNm) override;
  void commandTorquerMoment(int index, float momentAm2) override;

  // Last-applied commands, for the harness's own EPS power-draw
  // bookkeeping -- a real EPS monitor would read back commanded state as
  // its own telemetry, not intercept FlightSoftware's internal state (it no
  // longer returns a command struct at all).
  std::array<float, NUM_WHEELS> lastWheelTorqueNm{};
  std::array<float, NUM_TORQUERS> lastTorquerMomentAm2{};

private:
  Cubesat &sat_;
  glm::vec3 gravity_{0.0f};
  glm::vec3 ambientFieldWorld_{0.0f};
  glm::vec3 sunDirWorld_{0.0f};
  bool inEclipse_ = false;
};
