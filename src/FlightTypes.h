#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <array>

// Plain-data contract between ADCS (flight software) and whatever is
// driving it -- this simulation today, a HIL rig reading real ADC/I2C
// values later, real flight hardware eventually. No rigidbody/* includes
// here on purpose: this header, and everything ADCS.h/.cpp builds on top
// of it, must never know a RigidBody or PhysicsWorld exists. The
// simulation harness (satellite_adcs_sim.cpp) is the only place that
// bridges between this and the actual simulation objects.
//
// Fixed sizes rather than runtime-sized containers, matching the actual
// hardware config (a 4-wheel pyramid, 3 orthogonal magnetorquers) -- a
// real flight computer's hardware set is fixed at compile/init time, not
// queried live off simulation objects every cycle, and fixed-size arrays
// mean zero dynamic allocation in the flight code's hot path.
constexpr int NUM_WHEELS = 4;
constexpr int NUM_TORQUERS = 3;

// Pointing modes. Nadir/Sun/Target/Slew/Fine-pointing/Reflect all resolve
// to a target attitude and go through the normal PID/LQR/cascaded
// quaternion controller (with different tuning per mode -- see
// modeTuning() in ADCS.cpp); Detumble bypasses attitude control entirely
// and just damps rate, since a freshly-deployed, tumbling satellite has no
// attitude reference worth chasing yet.
//
// Lives here rather than in ADCS.h because FDIR.h needs it too (its
// evaluate() returns one) without creating an ADCS.h <-> FDIR.h include
// cycle -- same category of shared plain-data type as everything else in
// this header.
enum class PointingMode
{
  NADIR,         // body +Z toward a fixed "down" direction (no orbit in this sim, so no real nadir vector)
  SUN_POINTING,  // body +Z toward the sun
  DETUMBLE,      // damp angular rate toward zero; ignores attitude entirely
  TARGET,        // body +Z toward `target`, default tuning
  SLEW,          // body +Z toward `target`, tuned for a fast large-angle move
  FINE_POINTING, // body +Z toward `target`, tuned for a slow, precise settle
  REFLECT,       // body +Z (the mirror's normal) bisects sun and target -- bounces sunlight onto `target`
};

// ---------------------------------------------------------------------------
// Hardware configuration -- set once at ADCS::configure(), cached for the
// life of the program. Mirrors what a real flight computer would get from
// a compile-time/init-time hardware config table.
// ---------------------------------------------------------------------------
struct WheelConfig
{
  glm::vec3 spinAxisBody;
  float maxTorqueNm;
  float maxSpeedRadS;
  float wheelInertia; // kg*m^2
};

struct TorquerConfig
{
  glm::vec3 axisBody;
  float maxMomentAm2;
};

struct HardwareConfig
{
  std::array<WheelConfig, NUM_WHEELS> wheels;
  std::array<TorquerConfig, NUM_TORQUERS> torquers;
  glm::mat3 busInertiaTensor{1.0f}; // spacecraft bus inertia, for controller autoTune
};

// ---------------------------------------------------------------------------
// Sensor inputs -- one snapshot per ADCS cycle. Every "valid" flag models a
// real, routine failure mode (blinded, out of range, no lock) that flight
// software has to run through, not an edge case to ignore.
// ---------------------------------------------------------------------------
struct ImuSample
{
  glm::vec3 gyro{0.0f};  // rad/s, body frame
  glm::vec3 accel{0.0f}; // m/s^2, body frame (specific force)
};

struct MagSample
{
  glm::vec3 fieldBody{0.0f}; // Tesla
  bool valid = false;
};

struct StarTrackerSample
{
  glm::quat attitudeWorld{1, 0, 0, 0};
  bool valid = false;
};

struct SunSensorSample
{
  glm::vec3 sunDirBody{0.0f}; // unit vector, body frame
  bool valid = false;
};

struct WheelTelemetry
{
  float speedRadS = 0.0f;
  bool healthy = true;
};

// EPS (electrical power subsystem) telemetry -- what a real flight computer
// reads off its battery fuel gauge/charge controller, not something ADCS
// derives itself. No "valid" flag: unlike the sensors above, this sim
// doesn't model an EPS telemetry failure mode, so it's always available.
struct PowerSample
{
  float batterySoc = 1.0f;     // 0-1, state of charge
  float batteryVoltageV = 0.0f; // V
};

struct FSWInputs
{
  ImuSample imu;
  MagSample mag;
  StarTrackerSample star;
  SunSensorSample sunSensor;
  PowerSample power;
  std::array<WheelTelemetry, NUM_WHEELS> wheelTelemetry{};

  // Stands in for a real navigation solution (GPS/ephemeris) -- needed by
  // TARGET/SUN_POINTING guidance to compute a pointing direction relative
  // to the spacecraft's own position. This sim's harness feeds it straight
  // from simulation truth (no real nav modeled), same category of
  // simplification as elsewhere in this project.
  glm::vec3 spacecraftPositionWorld{0.0f};
};

// ---------------------------------------------------------------------------
// Actuator commands -- ADCS's output each cycle. The harness (or, later, a
// HIL adapter) is responsible for actually applying these to hardware.
// ---------------------------------------------------------------------------
struct WheelCommand
{
  float torqueNm = 0.0f;
};

struct TorquerCommand
{
  float momentAm2 = 0.0f;
};

struct FSWOutputs
{
  std::array<WheelCommand, NUM_WHEELS> wheelCommands{};
  std::array<TorquerCommand, NUM_TORQUERS> torquerCommands{};
};
