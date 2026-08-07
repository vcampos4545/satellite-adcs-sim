#pragma once
#include "FlightTypes.h"

// Every FlightSoftware task's own period, matching how a real bare-metal
// main loop's tasks are each rate-gated independently rather than all
// running at one shared cycle rate. Wheel telemetry has no period of its
// own -- it's read directly inside the ADCS task (see FlightSoftware::step()),
// the same way a real wheel-driver telemetry read is usually coupled to the
// control task that commands it, not sampled independently.
struct FswSchedule
{
  float imuPeriodS = 0.01f;        // 100 Hz -- representative MEMS gyro rate
  float magPeriodS = 0.05f;        // 20 Hz
  float starTrackerPeriodS = 0.2f; // 5 Hz -- image-processing-bound, real trackers solve slower than IMU/mag
  float sunSensorPeriodS = 0.1f;   // 10 Hz -- cheap analog read, no heavy processing
  float powerPeriodS = 1.0f;       // 1 Hz -- EPS fuel-gauge/charge-controller telemetry, a separate slower subsystem
  float navPeriodS = 1.0f;         // 1 Hz -- onboard nav solution update rate
  float adcsPeriodS = 0.05f;       // 20 Hz -- the attitude estimation/guidance/control/FDIR cycle itself
};

// Pull-based sensor/actuator interface FlightSoftware::step() drives on its
// own schedule (see FswSchedule) -- the literal HAL boundary a HIL adapter
// would implement against real ADC/I2C/SPI reads and driver writes instead
// of simulated hardware. Hardware-abstracted the same way every other type
// in src/fsw/ is: only plain FlightTypes.h data crosses this interface,
// never a RigidBody/PhysicsWorld/simulated sensor or actuator object.
class FlightSoftwareHAL
{
public:
  virtual ~FlightSoftwareHAL() = default;

  // Sensor reads. IMU/mag take `dt` (seconds since this sensor was last
  // read) because their underlying noise/bias-random-walk models are
  // dt-integrated (matches IMU::sample()/Magnetometer::sample()'s own
  // signatures in spacecraft-dynamics-sim) -- FlightSoftware passes the
  // actual elapsed time it tracked, not an assumed constant, so a
  // slightly-late tick still integrates noise/bias correctly. Star
  // tracker/sun sensor/power/nav are instantaneous reads of current state
  // (their underlying sample() calls take no dt either).
  virtual ImuSample readImu(float dt) = 0;
  virtual MagSample readMag(float dt) = 0;
  virtual StarTrackerSample readStarTracker() = 0;
  virtual SunSensorSample readSunSensor() = 0;
  virtual PowerSample readPower() = 0;
  virtual std::array<WheelTelemetry, NUM_WHEELS> readWheelTelemetry() = 0;
  // Stands in for a real navigation solution (GPS/onboard ephemeris) -- see
  // FlightTypes.h's spacecraftPositionWorld comment for why FSW needs this.
  virtual glm::vec3 readNavPosition() = 0;

  // Actuator writes -- issued directly, not returned as a struct for the
  // caller to apply later, matching how real firmware calls a driver
  // write() function in the same loop iteration it computed the command.
  virtual void commandWheelTorque(int index, float torqueNm) = 0;
  virtual void commandTorquerMoment(int index, float momentAm2) = 0;
};
