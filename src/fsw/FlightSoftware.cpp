#include "FlightSoftware.h"
#include "core/Satellite.h"
#include "core/GroundStations.h"
#include "core/Config.h"
#include <rigidbody/orbit/OrbitFrames.h>
#include <rigidbody/sensors/IMU.h>
#include <rigidbody/sensors/Magnetometer.h>
#include <rigidbody/sensors/StarTracker.h>
#include <rigidbody/sensors/SunSensor.h>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>
#include <cmath>

FlightSoftware::FlightSoftware(Satellite &spacecraft) : spacecraft_(spacecraft)
{
}

void FlightSoftware::configure(const HardwareConfig &hw, const glm::quat &initialAttitude)
{
  adcs.configure(hw, initialAttitude);
}

void FlightSoftware::step(float dt, const glm::dvec3 &satEciPosition, double currentJdNow,
                          const glm::vec3 &sunPositionWorld, const glm::vec3 &ambientFieldWorld,
                          bool inEclipse)
{
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
      satEciPosition, OrbitFrames::gmstRad(currentJdNow),
      glm::radians(static_cast<double>(Config::GROUND_STATION_MIN_ELEVATION_DEG)), targetEci);
  adcs.targetValid = (chosenGroundStation != nullptr);
  if (chosenGroundStation)
    adcs.target = glm::vec3(targetEci);
  if (chosenGroundStation != selectedGroundStation_)
  {
    selectedGroundStation_ = chosenGroundStation;
    adcs.resetController(); // clear integral windup from the previous target
  }

  adcs.sunPosition = sunPositionWorld;
  adcs.ambientFieldWorld = ambientFieldWorld;

  spacecraft_.gravity = glm::vec3(0.0f); // no Gravity generator attached (free-floating orbit)
  spacecraft_.ambientFieldWorld = ambientFieldWorld;
  spacecraft_.sunDirWorld = sunPositionWorld - spacecraft_.body->position;
  spacecraft_.inEclipse = inEclipse;

  // Sample each sensor independently, at its own realistic firmware rate
  // -- see this class's own header comment. IMU/magnetometer/wheel
  // telemetry are fast enough in reality to keep up with this loop's own
  // rate, so they're sampled fresh every cycle below.
  IMU::Reading imuR = spacecraft_.imu.sample(*spacecraft_.body, spacecraft_.gravity, dt);
  ImuSample imu{imuR.gyro, imuR.accel};

  Magnetometer::Reading magR = spacecraft_.magnetometer.sample(*spacecraft_.body, spacecraft_.ambientFieldWorld, dt);
  MagSample mag{magR.field, true};

  std::array<WheelTelemetry, NUM_WHEELS> wheelTelemetry;
  for (int i = 0; i < NUM_WHEELS; i++)
    wheelTelemetry[i] = {spacecraft_.wheels[i]->currentSpeed, spacecraft_.wheels[i]->healthFactor > 0.01f};

  // Star tracker: genuinely slower real hardware (image-processing-based
  // attitude solve) -- only a cycle where its own timer has actually
  // elapsed gets a fresh frame; every other cycle reports no new reading
  // (valid=false) rather than resend the last one, so ADCS never
  // repeatedly re-corrects against a now-stale absolute attitude (see
  // FlightSoftware.h's own header comment).
  StarTrackerSample star;
  starTrackerTimer_ += dt;
  if (starTrackerTimer_ > Config::STAR_TRACKER_SAMPLE_PERIOD_S)
  {
    starTrackerTimer_ = 0.0f;
    StarTracker::Reading r = spacecraft_.starTracker.sample(*spacecraft_.body, spacecraft_.sunDirWorld);
    star = {r.attitude, r.valid};
  }

  // Sun sensor: also genuinely slower than this loop in reality (a coarse
  // analog sensor polled over a bus), but unlike the star tracker a stale
  // reading is harmless to keep using -- it's only ever a TRIAD reference
  // vector, and the sun's apparent direction barely moves over one sample
  // period -- so the last sample simply persists until the next one.
  sunSensorTimer_ += dt;
  if (sunSensorTimer_ > Config::SUN_SENSOR_SAMPLE_PERIOD_S)
  {
    sunSensorTimer_ = 0.0f;
    SunSensor::Reading r = spacecraft_.sunSensor.sample(*spacecraft_.body, spacecraft_.sunDirWorld);
    // SunSensor has no eclipse model of its own (see its header: "valid is
    // always true here... a scenario that wants eclipse-aware dropouts
    // needs to gate this externally") -- a real coarse sun sensor reports
    // no lock when the sun itself is physically blocked by Earth, so that
    // blindness is applied here, on sample. This also keeps TRIAD fallback
    // (ADCS::computeTriadFallback) honest during eclipse: it already
    // requires a valid sun-sensor reading, but without this gate it would
    // happily TRIAD off a sun direction the satellite couldn't actually
    // observe.
    lastSunSensor_ = {r.sunDirBody, r.valid && !spacecraft_.inEclipse};
  }

  // Battery/power telemetry: a fuel-gauge IC's own slow polling rate,
  // reused between samples the same way as the sun sensor above.
  powerTimer_ += dt;
  if (powerTimer_ > Config::POWER_SAMPLE_PERIOD_S)
  {
    powerTimer_ = 0.0f;
    lastPower_ = {spacecraft_.battery.stateOfCharge(), spacecraft_.battery.voltage()};
  }

  glm::vec3 spacecraftPositionWorld = spacecraft_.body->position; // real orbital position (ECI meters) -- stands in for a real nav solution

  // Sense: attitude/rate estimation, published as ADCS's own telemetry.
  adcs.updateEstimator(imu, mag, star, lastSunSensor_, lastPower_, spacecraftPositionWorld, dt);

  // Evaluate health: FDIR reads exactly the FSW-derived signals that exist
  // at this point (wheel health telemetry, the EKF's own confidence, and
  // the bias-corrected rate implicit in gyroBiasEstimate) -- never
  // anything neither class could otherwise know. Runs even under
  // adcs.manualOverride (a fault is still worth detecting/logging while a
  // human has the stick); adcs.control() below is what actually respects
  // manualOverride and skips acting on it.
  FdirInputs fdirIn;
  fdirIn.wheelTelemetry = wheelTelemetry;
  fdirIn.attitudeUncertaintyDeg = adcs.attitudeUncertaintyDeg;
  fdirIn.rateBody = adcs.lastGyroBody - adcs.gyroBiasEstimate;
  fdirIn.batterySoc = adcs.batterySoc;
  fdirIn.commandedMode = adcs.mode;
  adcs.effectiveMode = fdir.evaluate(fdirIn, dt);

  // Act: guidance + control + allocation, executing whatever FDIR just
  // resolved effectiveMode to. Leaves the result in adcs.wheelCommands/
  // magnetorquerCommands -- applied to spacecraft_ directly below, the
  // same independence from ADCS that sensor sampling above has (see this
  // class's own header comment).
  adcs.control(wheelTelemetry, spacecraftPositionWorld, dt);

  for (int i = 0; i < NUM_WHEELS; i++)
    spacecraft_.wheels[i]->commandTorque(adcs.wheelCommands[i]);
  for (int i = 0; i < NUM_TORQUERS; i++)
    spacecraft_.magnetorquers[i]->commandDipoleMoment(adcs.magnetorquerCommands[i]);

  netPowerW = spacecraft_.updatePower(dt);

  // Ground-truth diagnostic -- computed here (not by ADCS, which has no
  // way to know spacecraft_.body->orientation), the same way this
  // project's true-vs-estimated pointing error comparisons always required
  // direct simulation access.
  glm::quat trueErrQ = glm::inverse(spacecraft_.body->orientation) * adcs.targetAttitude;
  if (trueErrQ.w < 0.0f)
    trueErrQ = -trueErrQ;
  trueErrDeg = glm::degrees(2.0f * std::acos(glm::clamp(trueErrQ.w, -1.0f, 1.0f)));
}

SystemMode FlightSoftware::systemMode() const
{
  return fdir.state() == FdirState::SAFE_HOLD ? SystemMode::SAFE : SystemMode::NOMINAL;
}
