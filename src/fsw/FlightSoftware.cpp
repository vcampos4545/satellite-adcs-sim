#include "FlightSoftware.h"

void FlightSoftware::configure(const HardwareConfig &hw, const glm::quat &initialAttitude)
{
  adcs.configure(hw, initialAttitude);
}

FSWOutputs FlightSoftware::step(const FSWInputs &in, float dt)
{
  // Sense: attitude/rate estimation, published as ADCS's own telemetry.
  adcs.updateEstimator(in, dt);

  // Evaluate health: FDIR reads exactly the FSW-derived signals that
  // exist at this point (wheel health telemetry, the EKF's own
  // confidence, and the bias-corrected rate implicit in
  // gyroBiasEstimate) -- never anything neither class could otherwise
  // know. Runs even under adcs.manualOverride (a fault is still worth
  // detecting/logging while a human has the stick); adcs.control() below
  // is what actually respects manualOverride and skips acting on it.
  FdirInputs fdirIn;
  fdirIn.wheelTelemetry = in.wheelTelemetry;
  fdirIn.attitudeUncertaintyDeg = adcs.attitudeUncertaintyDeg;
  fdirIn.rateBody = in.imu.gyro - adcs.gyroBiasEstimate;
  fdirIn.batterySoc = adcs.batterySoc;
  fdirIn.commandedMode = adcs.mode;
  adcs.effectiveMode = fdir.evaluate(fdirIn, dt);

  // Act: guidance + control + allocation, executing whatever FDIR just
  // resolved effectiveMode to.
  return adcs.control(in, dt);
}

SystemMode FlightSoftware::systemMode() const
{
  return fdir.state() == FdirState::SAFE_HOLD ? SystemMode::SAFE : SystemMode::NOMINAL;
}
