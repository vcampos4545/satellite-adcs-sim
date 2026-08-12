#pragma once
#include "ADCS.h"
#include "FDIR.h"

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

// This project's actual cyclic FSW entry point -- sensor data in, actuator
// commands out, once per call. Owns ADCS (GNC: attitude estimation,
// guidance, control, actuator allocation) and FDIR (fault detection/mode
// override) as *peers*, not FDIR nested inside ADCS -- matching real
// flight-software convention (NASA cFS, JPL's F', most smallsat stacks all
// keep fault/health monitoring as its own module sitting above or beside
// the subsystems it watches, not embedded in one subsystem's own control
// loop). FDIR is evaluated first each cycle, using ADCS's freshly-updated
// telemetry (attitude uncertainty, bias-corrected rate -- see
// ADCS::updateEstimator()), then ADCS is told what to actually execute
// (ADCS::control()) -- the "sense -> evaluate health -> act" ordering a
// real cyclic executive follows.
//
// A pure function of (internal state, FSWInputs, dt) -> FSWOutputs, same
// as ADCS::step() itself -- deliberately no HAL/interface layer here.
// Whoever calls this (this project's harness today, a HIL rig eventually)
// is responsible for building FSWInputs from real sensors and applying the
// returned FSWOutputs to real actuators; see Satellite::sampleSensors()/
// applyActuatorCommands() for how this simulation does that.
class FlightSoftware
{
public:
  ADCS adcs;
  FDIR fdir;

  void configure(const HardwareConfig &hw, const glm::quat &initialAttitude);

  FSWOutputs step(const FSWInputs &in, float dt);

  SystemMode systemMode() const;
};
