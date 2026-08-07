#include "FlightSoftware.h"

void FlightSoftware::configure(const HardwareConfig &hw, const FswSchedule &schedule,
                                const glm::quat &initialAttitude, FlightSoftwareHAL &hal)
{
  adcs.configure(hw, initialAttitude);
  schedule_ = schedule;
  hal_ = &hal;

  // Prime the scheduler: start every timer at its own period so the very
  // first step() call triggers an initial read of every sensor, instead of
  // waiting a full period and running the first ADCS cycle against
  // all-default-constructed samples.
  imuTimer_ = schedule_.imuPeriodS;
  magTimer_ = schedule_.magPeriodS;
  starTimer_ = schedule_.starTrackerPeriodS;
  sunSensorTimer_ = schedule_.sunSensorPeriodS;
  powerTimer_ = schedule_.powerPeriodS;
  navTimer_ = schedule_.navPeriodS;
  adcsTimer_ = 0.0f; // the ADCS task should still wait for its own first full period, once every sensor has an initial reading
}

bool FlightSoftware::step(float dt)
{
  // Each sensor task: rate-gated against its own configured period,
  // independent of every other task -- the asynchronous multi-rate
  // scheduling shape a real bare-metal firmware main loop has (an IMU task
  // running much faster than a star-tracker task, etc.), not one shared
  // cycle rate for everything.
  imuTimer_ += dt;
  if (imuTimer_ >= schedule_.imuPeriodS)
  {
    latestImu_ = hal_->readImu(imuTimer_);
    imuTimer_ = 0.0f;
  }

  magTimer_ += dt;
  if (magTimer_ >= schedule_.magPeriodS)
  {
    latestMag_ = hal_->readMag(magTimer_);
    magTimer_ = 0.0f;
  }

  starTimer_ += dt;
  if (starTimer_ >= schedule_.starTrackerPeriodS)
  {
    latestStar_ = hal_->readStarTracker();
    starTimer_ = 0.0f;
  }

  sunSensorTimer_ += dt;
  if (sunSensorTimer_ >= schedule_.sunSensorPeriodS)
  {
    latestSunSensor_ = hal_->readSunSensor();
    sunSensorTimer_ = 0.0f;
  }

  powerTimer_ += dt;
  if (powerTimer_ >= schedule_.powerPeriodS)
  {
    latestPower_ = hal_->readPower();
    powerTimer_ = 0.0f;
  }

  navTimer_ += dt;
  if (navTimer_ >= schedule_.navPeriodS)
  {
    latestNavPosition_ = hal_->readNavPosition();
    navTimer_ = 0.0f;
  }

  // The ADCS task: estimate -> evaluate health -> act, using whatever the
  // latest cached reading from each sensor task above happens to be right
  // now (the same "read whatever the last async task produced" semantics a
  // real multi-rate scheduler has).
  adcsTimer_ += dt;
  if (adcsTimer_ < schedule_.adcsPeriodS)
    return false;
  adcsTimer_ = 0.0f;

  FSWInputs in;
  in.imu = latestImu_;
  in.mag = latestMag_;
  in.star = latestStar_;
  in.sunSensor = latestSunSensor_;
  in.power = latestPower_;
  // Coupled to the ADCS task itself rather than its own schedule entry --
  // see FswSchedule's header comment for why.
  in.wheelTelemetry = hal_->readWheelTelemetry();
  in.spacecraftPositionWorld = latestNavPosition_;

  // Sense: attitude/rate estimation, published as ADCS's own telemetry.
  adcs.updateEstimator(in, schedule_.adcsPeriodS);

  // Evaluate health: FDIR reads exactly the FSW-derived signals that exist
  // at this point (wheel health telemetry, the EKF's own confidence, and
  // the bias-corrected rate implicit in gyroBiasEstimate) -- never anything
  // neither class could otherwise know. Runs even under
  // adcs.manualOverride (a fault is still worth detecting/logging while a
  // human has the stick); adcs.control() below is what actually respects
  // manualOverride and skips acting on it.
  FdirInputs fdirIn;
  fdirIn.wheelTelemetry = in.wheelTelemetry;
  fdirIn.attitudeUncertaintyDeg = adcs.attitudeUncertaintyDeg;
  fdirIn.rateBody = in.imu.gyro - adcs.gyroBiasEstimate;
  fdirIn.batterySoc = adcs.batterySoc;
  fdirIn.commandedMode = adcs.mode;
  adcs.effectiveMode = fdir.evaluate(fdirIn, schedule_.adcsPeriodS);

  // Act: guidance + control + allocation, executing whatever FDIR just
  // resolved effectiveMode to, then push the resulting commands straight
  // out through the HAL -- no struct returned for someone else to apply
  // later, matching how real firmware calls a driver write() in the same
  // loop iteration it computed the command.
  FSWOutputs out = adcs.control(in, schedule_.adcsPeriodS);
  for (int i = 0; i < NUM_WHEELS; i++)
    hal_->commandWheelTorque(i, out.wheelCommands[i].torqueNm);
  for (int i = 0; i < NUM_TORQUERS; i++)
    hal_->commandTorquerMoment(i, out.torquerCommands[i].momentAm2);

  return true;
}

SystemMode FlightSoftware::systemMode() const
{
  return fdir.state() == FdirState::SAFE_HOLD ? SystemMode::SAFE : SystemMode::NOMINAL;
}
