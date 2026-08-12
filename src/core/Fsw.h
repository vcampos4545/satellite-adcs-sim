#pragma once
#include "fsw/FlightSoftware.h"
#include "SimulationState.h"
#include "GroundStations.h"

// This project's flight-software cycle, wrapping FlightSoftware.h's
// hardware-abstracted core with everything the harness does around it each
// cycle: ground-station target selection, translating SimulationState's
// spacecraft to/from FSW's plain-data contract (Cubesat::sampleSensors()/
// applyActuatorCommands()), EPS accounting, and telemetry. FlightSoftware
// itself never sees `sim` -- only this class bridges the two.
class Fsw
{
public:
  FlightSoftware flightSoftware;
  float trueErrDeg = 0.0f; // ground-truth (sim-truth vs. FSW-estimate) pointing error, diagnostic only

  explicit Fsw(SimulationState &sim);

  void step(float dt);

private:
  SimulationState &sim_;
  const GroundStation *selectedGroundStation_ = nullptr;
};
