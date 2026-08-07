#include "FDIR.h"
#include <algorithm>

PointingMode FDIR::evaluate(const FdirInputs &in, float dt)
{
  missionTimeS_ += dt;

  uint32_t detected = FDIR_FAULT_NONE;

  int healthyWheels = 0;
  for (const WheelTelemetry &wt : in.wheelTelemetry)
    if (wt.healthy)
      healthyWheels++;
  if (healthyWheels < minHealthyWheels)
    detected |= FDIR_FAULT_WHEEL_AUTHORITY_LOST;

  // Sustained, not instantaneous -- a single noisy cycle above threshold
  // (or one dropped star-tracker frame) shouldn't be enough to trip a
  // fault; a real dropout that's actually eroding the estimate will still
  // be there `attitudeUncertaintySustainedS` later.
  if (in.attitudeUncertaintyDeg > attitudeUncertaintyTriggerDeg)
    uncertaintyElevatedTimerS_ += dt;
  else
    uncertaintyElevatedTimerS_ = 0.0f;
  if (uncertaintyElevatedTimerS_ > attitudeUncertaintySustainedS)
    detected |= FDIR_FAULT_ATTITUDE_UNCERTAIN;

  if (glm::length(in.rateBody) > excessRateRadS)
    detected |= FDIR_FAULT_EXCESS_RATE;

  if (in.batterySoc < lowBatterySocTrigger)
    detected |= FDIR_FAULT_LOW_BATTERY;

  // Latch: once tripped, a fault stays active even if the underlying
  // condition clears on its own, until clearLatchedFaults() explicitly
  // acknowledges it -- see the header comment on why.
  uint32_t newlyTripped = detected & ~latchedFaults_;
  if (newlyTripped != FDIR_FAULT_NONE)
    logEvent(newlyTripped, true);
  latchedFaults_ |= detected;
  activeFaults_ = latchedFaults_;

  if (!enabled || activeFaults_ == FDIR_FAULT_NONE)
  {
    state_ = FdirState::NOMINAL;
    return in.commandedMode;
  }

  state_ = FdirState::SAFE_HOLD;

  // Excess rate wins regardless of what else is active -- SUN_POINTING's
  // controller wasn't tuned for a rate outside its own envelope and won't
  // converge cleanly against it; damp the rate first, same reasoning
  // DETUMBLE already uses standalone.
  if (activeFaults_ & FDIR_FAULT_EXCESS_RATE)
    return PointingMode::DETUMBLE;

  // Otherwise: the closest thing this project models to a real safe mode
  // -- stop trying to do the mission, hold a stable, low-precision
  // attitude that doesn't depend on the condition that tripped (sun
  // direction is independent of wheel count and tolerant of a noisier
  // attitude estimate than fine pointing needs). For a low-battery fault
  // specifically this is also literally the correct real-world response,
  // not just a fallback: sun-pointing maximizes solar generation, the same
  // reason real spacecraft "safe mode" almost always means sun-pointing.
  return PointingMode::SUN_POINTING;
}

void FDIR::clearLatchedFaults()
{
  if (latchedFaults_ != FDIR_FAULT_NONE)
    logEvent(latchedFaults_, false);
  latchedFaults_ = FDIR_FAULT_NONE;
  activeFaults_ = FDIR_FAULT_NONE;
  uncertaintyElevatedTimerS_ = 0.0f;
  state_ = FdirState::NOMINAL;
}

const FdirEvent &FDIR::recentEvent(int i) const
{
  int idx = (eventWriteIdx_ - 1 - i) % EVENT_LOG_CAPACITY;
  if (idx < 0)
    idx += EVENT_LOG_CAPACITY;
  return eventLog[idx];
}

void FDIR::logEvent(uint32_t flags, bool entering)
{
  eventLog[eventWriteIdx_ % EVENT_LOG_CAPACITY] = {missionTimeS_, flags, entering};
  eventWriteIdx_++;
  eventCount = std::min(eventCount + 1, EVENT_LOG_CAPACITY);
}
