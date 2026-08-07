#pragma once
#include "fsw/ADCS.h"
#include <rigidbody/PhysicsWorld.h>
#include <rigidbody/actuators/ReactionWheel.h>
#include <vector>

// Simulation-level knobs, as opposed to ADCS/FSW state -- things that
// belong to the test harness around the spacecraft, not to the spacecraft
// itself. Held by value in main() and handed to drawSimulationTab() by
// reference each frame, same pattern as ADCS's own public fields.
//
// Time controls (paused/timeScale/stepRequested): main()'s simulation
// block advances by `simDt = realDt * timeScale` each frame while
// !paused; while paused, a single step of Config::TIME_STEP_S is taken
// instead if stepRequested is set (then main() clears it back to false --
// this is a one-shot request, not a held state). timeScale applies
// uniformly to orbit propagation, physics, and the FSW cycle timer
// together, not just wall-clock display, so relative rates between them
// (e.g. how many ADCS cycles happen per orbit) stay physically correct
// at any speed -- see main()'s own comment on why the FSW cycle loop is a
// `while`, not an `if`, once timeScale can make simDt span multiple
// nominal ADCS cycles in one frame.
struct SimControls
{
  bool paused = false;
  float timeScale = 1.0f;
  bool stepRequested = false;
  float tumbleKickRadS;

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
