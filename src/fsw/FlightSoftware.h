#pragma once
#include "ADCS.h"
#include "FDIR.h"

struct Satellite;
struct GroundStation;

enum class SystemMode
{
  SAFE,    // an active fault is forcing safe, low-precision pointing
  NOMINAL, // no active fault; commanded mode executes as requested
};

// This project's actual cyclic FSW entry point -- once per call, drives one
// full ADCS/FDIR cycle against a real Satellite and does everything else
// this project's simulation needs around that: ground-station target
// selection, environment wiring (sun/field/eclipse), sensor sampling,
// actuator commanding, EPS accounting. Owns ADCS (GNC: attitude
// estimation, guidance, control, actuator allocation) and FDIR (fault
// detection/mode override) as *peers*, not FDIR nested inside ADCS --
// matching real flight-software convention (NASA cFS, JPL's F', most
// smallsat stacks all keep fault/health monitoring as its own module
// sitting above or beside the subsystems it watches, not embedded in one
// subsystem's own control loop). FDIR is evaluated first each cycle, using
// ADCS's freshly-updated telemetry (attitude uncertainty, bias-corrected
// rate -- see ADCS::updateEstimator()), then ADCS is told what to actually
// execute (ADCS::control()) -- the "sense -> evaluate health -> act"
// ordering a real cyclic executive follows.
//
// This is the one place simulated hardware is sampled/commanded --
// deliberately not a Satellite method (see its own header comment): each
// sensor is read independently, at its own realistic firmware rate, not
// as one bundled per-cycle snapshot. IMU, magnetometer, and wheel
// telemetry are fast enough in reality to keep up with this loop's own
// rate, so step() samples them fresh every cycle; the star tracker, sun
// sensor, and battery/power telemetry are genuinely slower real hardware
// (an image-processing-based attitude solve, a coarse analog sensor
// polled over a bus, an I2C fuel-gauge IC), so each is gated behind its
// own timer (see the private fields below) -- on a cycle without a fresh
// reading, ADCS is simply told so (see ADCS::updateEstimator()'s own
// comment on treating that as a routine dropout), the same way real
// flight software polls sensors that aren't always ready with something
// new.
class FlightSoftware
{
public:
  ADCS adcs;
  FDIR fdir;

  // Ground-truth (sim-truth vs. FSW-estimate) pointing error, diagnostic
  // only -- computed here (not by ADCS, which has no way to know
  // spacecraft_.body->orientation).
  float trueErrDeg = 0.0f;

  // Net EPS power (W) from this cycle's Satellite::updatePower() call --
  // positive charges, negative discharges. Published as a member (same
  // pattern as trueErrDeg above) rather than returned from step(): this
  // class has no UI-plotting concern of its own, so the caller reads this
  // plus adcs's public telemetry fields to build whatever history it wants.
  float netPowerW = 0.0f;

  explicit FlightSoftware(Satellite &spacecraft);

  void configure(const HardwareConfig &hw, const glm::quat &initialAttitude);

  // One FSW cycle. `satEciPosition`/`currentJdNow` are only needed for
  // ground-station target selection (the satellite's real orbital
  // position/current time -- stands in for a real navigation/clock
  // solution); `sunPositionWorld`/`ambientFieldWorld`/`inEclipse` are this
  // cycle's environment, wired into both ADCS's guidance references and
  // the spacecraft's own sensor/EPS models before they're sampled.
  void step(float dt, const glm::dvec3 &satEciPosition, double currentJdNow,
            const glm::vec3 &sunPositionWorld, const glm::vec3 &ambientFieldWorld,
            bool inEclipse);

  SystemMode systemMode() const;

private:
  Satellite &spacecraft_;
  const GroundStation *selectedGroundStation_ = nullptr;

  // Per-sensor sample-rate timers (see this class's own header comment)
  // and the last reading each throttled sensor produced -- reused on
  // cycles that don't resample. The star tracker deliberately has no
  // "last reading" cache: a stale absolute-attitude correction fed again
  // on a later cycle would repeatedly snap the EKF back to an attitude
  // the body has since rotated away from, actively fighting propagation,
  // so a cycle without a fresh frame reports valid=false instead (see
  // ADCS::updateEstimator()) rather than resend the old one. The sun
  // sensor and battery telemetry have no such correction risk -- they're
  // read as ambient reference/state, not corrected against once -- so
  // reusing a several-cycles-stale value is harmless and realistic.
  float starTrackerTimer_ = 0.0f;
  float sunSensorTimer_ = 0.0f;
  float powerTimer_ = 0.0f;
  SunSensorSample lastSunSensor_;
  PowerSample lastPower_;
};
