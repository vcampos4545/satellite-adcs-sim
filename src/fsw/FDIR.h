#pragma once
#include "FlightTypes.h"
#include <glm/glm.hpp>
#include <array>
#include <cstdint>

// Autonomous Fault Detection, Isolation, and Recovery: a small mode-manager
// state machine that sits above the normal guidance/control loop and
// decides, every cycle, whether the spacecraft is healthy enough to keep
// doing whatever pointing mode is currently commanded, or whether it needs
// to step in and force a safer one -- the same job a real FSW's "mode
// manager"/"health monitor" task does, just scoped to what this project's
// sensor/actuator set can actually detect (wheel health telemetry, EKF
// confidence, body rate, and battery state of charge; nothing about
// thermal/comm, since none of that is modeled here).
//
// Hardware-abstracted like ADCS (see FlightTypes.h): FdirInputs is plain
// data, evaluate() returns a plain PointingMode recommendation. No dynamic
// allocation -- the event log is a fixed-size ring buffer, same convention
// ADCS itself follows.

// Which specific condition tripped -- a bitmask (not a single enum value)
// so more than one can be active/logged at once, e.g. a wheel loss and
// elevated uncertainty together. Named after real ops usage: this is what
// a fault log/ground op would call a "fault code."
enum FdirFaultFlags : uint32_t
{
  FDIR_FAULT_NONE = 0,
  FDIR_FAULT_WHEEL_AUTHORITY_LOST = 1u << 0, // fewer than minHealthyWheels healthy -- pyramid can't reliably span all 3 axes anymore
  FDIR_FAULT_ATTITUDE_UNCERTAIN = 1u << 1,   // EKF confidence too low for too long -- no attitude worth controlling against
  FDIR_FAULT_EXCESS_RATE = 1u << 2,          // body rate outside the attitude controller's design envelope
  FDIR_FAULT_LOW_BATTERY = 1u << 3,          // state of charge below lowBatterySocTrigger
};

enum class FdirState
{
  NOMINAL,   // no active fault; commanded mode passes through unmodified
  SAFE_HOLD, // an active fault is forcing a safe mode in place of the commanded one
};

struct FdirInputs
{
  std::array<WheelTelemetry, NUM_WHEELS> wheelTelemetry{};
  float attitudeUncertaintyDeg = 0.0f;
  glm::vec3 rateBody{0.0f};                          // bias-corrected body rate (rad/s)
  float batterySoc = 1.0f;                           // 0-1, state of charge
  PointingMode commandedMode = PointingMode::TARGET; // what ground/UI last asked for
};

// One entry in the fault log: what tripped (or cleared) and when. Not a
// full telemetry stream -- just enough for a UI panel or downlink to show
// "what happened and when," the same role a real fault log serves.
struct FdirEvent
{
  float missionTimeS = 0.0f;
  uint32_t flags = FDIR_FAULT_NONE;
  bool entering = true; // true: newly declared; false: cleared
};

class FDIR
{
public:
  // Ground-commandable inhibit -- real FSWs universally have one of these,
  // since autonomy that can't be turned off is a liability during
  // commissioning/testing on orbit, not just a simulation convenience.
  bool enabled = true;

  // Thresholds, exposed for a UI panel to tune live.
  int minHealthyWheels = 3;                     // pyramid needs >=3 non-degenerate spin axes for full 3-axis authority
  float attitudeUncertaintyTriggerDeg = 5.0f;    // 1-sigma uncertainty above this starts the sustain timer
  float attitudeUncertaintySustainedS = 5.0f;    // ...and it has to stay above that long before actually faulting
  float excessRateRadS = 2.0f;                   // body rate above this is outside the controller's tuned envelope
  float lowBatterySocTrigger = 0.2f;             // state of charge below this trips immediately -- SOC doesn't jitter like a sensor reading, no sustain needed

  static constexpr int EVENT_LOG_CAPACITY = 16;
  std::array<FdirEvent, EVENT_LOG_CAPACITY> eventLog{};
  int eventCount = 0; // number of valid entries in eventLog, capped at capacity

  FdirState state() const { return state_; }
  uint32_t activeFaults() const { return activeFaults_; }
  float missionTimeS() const { return missionTimeS_; }

  // Runs the fault checks and returns the mode the guidance/control loop
  // should actually execute this cycle: `in.commandedMode` unmodified while
  // NOMINAL, or an autonomously-selected safe mode while SAFE_HOLD.
  PointingMode evaluate(const FdirInputs &in, float dt);

  // Ground command: drops any latched faults regardless of whether their
  // triggering condition has actually cleared. Real FDIR latches faults
  // deliberately (a wheel that recovers on its own doesn't mean ops should
  // stop knowing it failed) -- this is the explicit acknowledgment that
  // un-latches them, the autonomous equivalent of "RESUME" after ground has
  // reviewed the event.
  void clearLatchedFaults();

  // Returns the i-th most recent event (0 = most recent), in write order --
  // for a UI panel that wants to show the log newest-first without the
  // caller reasoning about the ring buffer's wraparound itself.
  const FdirEvent &recentEvent(int i) const;

private:
  FdirState state_ = FdirState::NOMINAL;
  uint32_t activeFaults_ = FDIR_FAULT_NONE;
  uint32_t latchedFaults_ = FDIR_FAULT_NONE;
  float uncertaintyElevatedTimerS_ = 0.0f;
  float missionTimeS_ = 0.0f;
  int eventWriteIdx_ = 0;

  void logEvent(uint32_t flags, bool entering);
};
