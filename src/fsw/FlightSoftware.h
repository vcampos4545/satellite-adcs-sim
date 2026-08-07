#pragma once
#include "ADCS.h"
#include "FDIR.h"
#include "FlightSoftwareHAL.h"

// Top-level "what is the spacecraft doing right now" -- coarser than
// ADCS's own PointingMode (fine-grained attitude-control guidance),
// matching real flight software's own layered mode structure. Currently
// derived 1:1 from FDIR's NOMINAL/SAFE_HOLD state (FDIR is this
// project's only source of "should we still be doing the mission"
// today), but deliberately exposed under this stable, firmware-style
// name/API (FlightSoftware::systemMode(), not fdir.state()) rather than
// requiring every caller to know FDIR's own state enum -- so a future
// addition to what can force SAFE (e.g. a thermal fault, once thermal
// is modeled) doesn't change what callers have to know about.
enum class SystemMode
{
  SAFE,    // an active fault is forcing safe, low-precision pointing
  NOMINAL, // no active fault; commanded mode executes as requested
};

// The flight software's actual firmware main-loop body: step() is meant to
// be called every tick of a fast, real-time loop (see Config::FIRMWARE_TICK_S
// in the harness), the same shape a bare-metal (no RTOS) main() has -- a
// superloop where each task (read a sensor, run the attitude estimator/
// controller, write actuator commands) is rate-gated against its own
// configured period (FswSchedule) rather than everything running in lockstep
// at one shared rate. Sensor data is *pulled* on that schedule through an
// injected FlightSoftwareHAL, and actuator commands are *pushed* through it
// the moment they're computed -- matching how a real driver-based firmware
// loop calls hal->readX()/hal->commandY() on its own schedule, rather than
// receiving a pre-packaged input struct and returning a pre-packaged output
// struct once per call. This is also the seam a future HIL adapter would
// target: implement FlightSoftwareHAL against real hardware, and this class
// doesn't change at all.
//
// Owns ADCS (GNC: attitude estimation, guidance, control, actuator
// allocation) and FDIR (fault detection/mode override) as *peers*, not FDIR
// nested inside ADCS -- matching real flight-software convention (NASA cFS,
// JPL's F', most smallsat stacks all keep fault/health monitoring as its own
// module sitting above or beside the subsystems it watches, not embedded in
// one subsystem's own control loop). Within the ADCS task specifically, FDIR
// is evaluated first each time it runs, using ADCS's freshly-updated
// telemetry (attitude uncertainty, bias-corrected rate -- see
// ADCS::updateEstimator()), then ADCS is told what to actually execute
// (ADCS::control()) -- the "sense -> evaluate health -> act" ordering a real
// cyclic executive follows.
class FlightSoftware
{
public:
  ADCS adcs;
  FDIR fdir;

  void configure(const HardwareConfig &hw, const FswSchedule &schedule,
                 const glm::quat &initialAttitude, FlightSoftwareHAL &hal);

  // Call once per firmware tick. Internally rate-gates every sensor read
  // and the ADCS task itself against `schedule`'s periods, using the most
  // recently pulled reading for any task that hasn't fired yet this tick --
  // exactly how an asynchronous multi-rate firmware scheduler behaves.
  // Returns true iff the ADCS task (estimate -> FDIR -> guidance/control ->
  // actuator commands) actually ran this call, so a caller that needs to
  // know "did a new control cycle just happen" (e.g. this project's harness,
  // for EPS/telemetry bookkeeping) doesn't have to assume every call did
  // something.
  bool step(float dt);

  SystemMode systemMode() const;

private:
  FswSchedule schedule_;
  FlightSoftwareHAL *hal_ = nullptr;

  float imuTimer_ = 0.0f;
  float magTimer_ = 0.0f;
  float starTimer_ = 0.0f;
  float sunSensorTimer_ = 0.0f;
  float powerTimer_ = 0.0f;
  float navTimer_ = 0.0f;
  float adcsTimer_ = 0.0f;

  ImuSample latestImu_;
  MagSample latestMag_;
  StarTrackerSample latestStar_;
  SunSensorSample latestSunSensor_;
  PowerSample latestPower_;
  glm::vec3 latestNavPosition_{0.0f};
};
