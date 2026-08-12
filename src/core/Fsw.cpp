#include "Fsw.h"
#include "Config.h"
#include <rigidbody/orbit/OrbitFrames.h>

Fsw::Fsw(SimulationState &sim) : sim_(sim)
{
  flightSoftware.configure(sim_.hwConfig, sim_.spacecraft.body->orientation);
}

void Fsw::step(float dt)
{
  ADCS &adcs = flightSoftware.adcs;
  Cubesat &spacecraft = sim_.spacecraft;
  const OrbitState &orbitState = sim_.world.orbitalState(spacecraft.body);

  adcs.sunPosition = sim_.sunPositionNow;

  // Ground-station target selection: recomputed every FSW cycle (cheap --
  // 6 stations, a distance/elevation compare each), so adcs.target keeps
  // tracking the selected station's real rotating ECI position even
  // between actual handoffs. A station must be within the satellite's
  // footprint to be a viable target at all -- selectClosestGroundStation
  // returns nullptr rather than falling back to an out-of-view station, in
  // which case adcs.targetValid goes false and ADCS's own guidance falls
  // back to sun-relative pointing (see ADCS.h's targetValid comment).
  // Only an actual change of *what's* selected clears controller integral
  // windup, not every frame the target's rotating position moves.
  glm::dvec3 targetEci;
  const GroundStation *chosenGroundStation = selectClosestGroundStation(
      orbitState.position, OrbitFrames::gmstRad(sim_.currentJdNow),
      glm::radians(static_cast<double>(Config::GROUND_STATION_MIN_ELEVATION_DEG)), targetEci);
  adcs.targetValid = (chosenGroundStation != nullptr);
  if (chosenGroundStation)
    adcs.target = glm::vec3(targetEci);
  if (chosenGroundStation != selectedGroundStation_)
  {
    selectedGroundStation_ = chosenGroundStation;
    adcs.resetController(); // clear integral windup from the previous target
  }

  adcs.ambientFieldWorld = sim_.fieldNow;
  glm::vec3 sunDirWorld = sim_.sunPositionNow - spacecraft.body->position;
  spacecraft.gravity = glm::vec3(0.0f); // no Gravity generator attached (free-floating orbit)
  spacecraft.ambientFieldWorld = sim_.fieldNow;
  spacecraft.sunDirWorld = sunDirWorld;
  spacecraft.inEclipse = sim_.inEclipse;

  // The only place simulated hardware is translated to/from FSW's
  // plain-data contract -- see Cubesat::sampleSensors()/
  // applyActuatorCommands() and FlightSoftware.h's own header comment.
  // FlightSoftware::step() itself never touches `spacecraft`.
  FSWInputs in = spacecraft.sampleSensors(dt);
  FSWOutputs out = flightSoftware.step(in, dt);
  spacecraft.applyActuatorCommands(out);

  float netPowerW = spacecraft.updatePower(dt, out);
  sim_.telemetry.netPowerW.push(netPowerW);

  // Ground-truth diagnostic -- computed here (not by ADCS, which has no
  // way to know body->orientation), the same way this project's true-vs-
  // estimated pointing error comparisons always required direct
  // simulation access.
  glm::quat trueErrQ = glm::inverse(spacecraft.body->orientation) * adcs.targetAttitude;
  if (trueErrQ.w < 0.0f)
    trueErrQ = -trueErrQ;
  trueErrDeg = glm::degrees(2.0f * std::acos(glm::clamp(trueErrQ.w, -1.0f, 1.0f)));

  // Pushed once per FSW cycle (a new sensor reading actually exists), not
  // once per render frame.
  sim_.telemetry.gyroMagDegS.push(glm::degrees(glm::length(adcs.lastGyroBody)));
  sim_.telemetry.accelMagMs2.push(glm::length(adcs.lastAccelBody));
  sim_.telemetry.magFieldMagUt.push(glm::length(adcs.magFieldBody) * 1e6f);
  sim_.telemetry.estimatedPointingErrorDeg.push(adcs.estimatedPointingErrorDeg);
  sim_.telemetry.truePointingErrorDeg.push(trueErrDeg);
  sim_.telemetry.batterySocPct.push(spacecraft.battery.stateOfCharge() * 100.0f);
}
