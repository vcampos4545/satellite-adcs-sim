#pragma once
#include "fsw/ADCS.h"
#include <rigidbody/PhysicsWorld.h>
#include <rigidbody/actuators/ReactionWheel.h>
#include <vector>

// Simulation-level knobs, as opposed to ADCS/FSW state -- things that
// belong to the test harness around the spacecraft, not to the spacecraft
// itself. Held by value in main() and handed to drawSimulationTab() by
// reference each frame, same pattern as ADCS's own public fields.
struct SimControls
{
  float tumbleKickRadS;

  // Multiplies real elapsed time before main()'s fixed-step accumulator
  // sees it -- 1.0 = real-time, 0.0 = paused. The FSW/physics step itself
  // always advances by exactly Config::TIME_STEP_S regardless of this
  // value (so EKF/controller tuning stays valid at any speed); this only
  // changes how many of those fixed steps happen per real second. See
  // Config::FSW_TIMER_MAX_S for the per-frame cap that keeps a high value
  // here from stalling the render loop.
  float timeScale = 1.0f;

  // glfwSwapInterval(1) vs (0) -- live-toggleable rather than hardcoded so
  // it can be turned off later if uncapped rendering is ever needed (e.g.
  // to read true render performance off the FPS overlay instead of the
  // display's refresh rate). Applied once per frame in main().
  bool vsyncEnabled = true;

  explicit SimControls(float tumbleKickRadSIn) : tumbleKickRadS(tumbleKickRadSIn) {}
};

// Mission epoch (UTC calendar date/time that orbitState.missionTimeS = 0
// corresponds to), editable live from the Simulation tab -- changing it
// doesn't touch the orbit itself (orbitState keeps propagating exactly as
// it was), only what real Julian Date main()'s currentJdNow maps
// missionTimeS onto, which is what Sun direction (SunModel) and Earth's
// rotation (OrbitFrames::gmstRad, see OrbitRenderer.h's drawEarth) are
// actually computed from -- so this is genuinely "what historical/future
// date is this orbit happening on," not a simulation reset.
struct EpochControls
{
  int year = 2026;
  int month = 1;
  int day = 1;
  int hour = 0;
  int minute = 0;
  float second = 0.0f;
};

void drawSimulationTab(SimControls &sim, EpochControls &epoch,
                       std::vector<ReactionWheel *> &wheels, RigidBody *body, ADCS &adcs);
