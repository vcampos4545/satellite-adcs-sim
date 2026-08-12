// Regression tests for ADCS's automatic detumble entry/exit and AUTO
// actuator selection (src/fsw/ADCS.h/.cpp) -- see docs/ALGORITHMS.md's
// "Detumble" section. Drives ADCS directly (FDIR-agnostic, via ADCS::step())
// matching test_adcs_control.cpp's style -- this feature lives in ADCS
// itself, not FDIR.
#include "test_common.h"
#include "ADCS.h"

namespace
{
FSWInputs makeInputs(const glm::vec3 &gyro, bool wheelsHealthy = true, float wheelSpeedRadS = 0.0f)
{
  FSWInputs in;
  in.imu = {gyro, glm::vec3(0.0f)};
  in.mag = {glm::vec3(0, 0, 3e-5f), true};
  in.star = {glm::quat(1, 0, 0, 0), true};
  in.power = {1.0f, 8.4f};
  in.spacecraftPositionWorld = glm::vec3(0.0f);
  for (int w = 0; w < NUM_WHEELS; w++)
    in.wheelTelemetry[w] = {wheelSpeedRadS, wheelsHealthy};
  return in;
}
} // namespace

int main()
{
  // High initial rate, commanded mode TARGET -> auto-enters DETUMBLE.
  {
    ADCS adcs;
    HardwareConfig hw = makeTestHardwareConfig();
    adcs.configure(hw, glm::quat(1, 0, 0, 0));
    adcs.mode = PointingMode::TARGET;
    adcs.target = glm::vec3(0, 0, 5.0f);
    adcs.ambientFieldWorld = glm::vec3(0, 0, 3e-5f);

    glm::vec3 highRate(1.0f, 1.0f, 1.0f); // |rate| ~1.73 rad/s, above detumbleEntryRateRadS
    FSWInputs in = makeInputs(highRate);
    adcs.step(in, 0.05f);

    CHECK(adcs.mode == PointingMode::DETUMBLE,
          "High initial rate auto-commands mode -> DETUMBLE (mode = %d)", static_cast<int>(adcs.mode));
  }

  // Rate below entry threshold the whole time -> no false trigger.
  {
    ADCS adcs;
    HardwareConfig hw = makeTestHardwareConfig();
    adcs.configure(hw, glm::quat(1, 0, 0, 0));
    adcs.mode = PointingMode::TARGET;
    adcs.target = glm::vec3(0, 0, 5.0f);
    adcs.ambientFieldWorld = glm::vec3(0, 0, 3e-5f);

    glm::vec3 lowRate(0.05f, 0.02f, 0.01f); // well under detumbleEntryRateRadS
    FSWInputs in = makeInputs(lowRate);
    for (int i = 0; i < 100; i++)
      adcs.step(in, 0.05f);

    CHECK(adcs.mode == PointingMode::TARGET,
          "Rate below entry threshold never auto-triggers DETUMBLE (mode = %d)", static_cast<int>(adcs.mode));
  }

  // Exact boundary: a rate at SLEW's own commanded-rate cap (ModeTuning's
  // omega_max, 1.0 rad/s -- see ADCS.cpp's tuningForMode()) must NOT
  // false-trigger auto-detumble, since Controllers.cpp's own comment
  // confirms an aggressive slew genuinely pins body rate near that cap.
  {
    ADCS adcs;
    HardwareConfig hw = makeTestHardwareConfig();
    adcs.configure(hw, glm::quat(1, 0, 0, 0));
    adcs.mode = PointingMode::SLEW;
    adcs.target = glm::vec3(0, 0, -5.0f);
    adcs.ambientFieldWorld = glm::vec3(0, 0, 3e-5f);

    glm::vec3 slewCapRate(0.0f, 0.0f, 1.0f); // exactly SLEW's omega_max magnitude
    FSWInputs in = makeInputs(slewCapRate);
    adcs.step(in, 0.05f);

    CHECK(adcs.mode == PointingMode::SLEW,
          "Rate at SLEW's own omega_max (1.0 rad/s) does not false-trigger DETUMBLE (mode = %d)",
          static_cast<int>(adcs.mode));
  }

  // Auto-entered DETUMBLE, rate damps below the exit threshold -> reverts
  // to SUN_POINTING.
  {
    ADCS adcs;
    HardwareConfig hw = makeTestHardwareConfig();
    adcs.configure(hw, glm::quat(1, 0, 0, 0));
    adcs.mode = PointingMode::TARGET;
    adcs.ambientFieldWorld = glm::vec3(0, 0, 3e-5f);

    // Auto-enter.
    adcs.step(makeInputs(glm::vec3(1.0f, 1.0f, 1.0f)), 0.05f);
    CHECK(adcs.mode == PointingMode::DETUMBLE, "Auto-entered DETUMBLE before exit check");

    // Rate now reads as damped, below the exit threshold.
    adcs.step(makeInputs(glm::vec3(0.01f, 0.0f, 0.0f)), 0.05f);

    CHECK(adcs.mode == PointingMode::SUN_POINTING,
          "Auto-entered DETUMBLE exits to SUN_POINTING once rate damps below the exit threshold (mode = %d)",
          static_cast<int>(adcs.mode));
  }

  // Manually-commanded DETUMBLE (never auto-entered) is never auto-exited,
  // even with rate low the whole time.
  {
    ADCS adcs;
    HardwareConfig hw = makeTestHardwareConfig();
    adcs.configure(hw, glm::quat(1, 0, 0, 0));
    adcs.mode = PointingMode::DETUMBLE; // manually commanded, not auto-entered
    adcs.ambientFieldWorld = glm::vec3(0, 0, 3e-5f);

    FSWInputs in = makeInputs(glm::vec3(0.01f, 0.0f, 0.0f)); // already below exit threshold
    for (int i = 0; i < 100; i++)
      adcs.step(in, 0.05f);

    CHECK(adcs.mode == PointingMode::DETUMBLE,
          "Manually-commanded DETUMBLE is never auto-exited (mode = %d)", static_cast<int>(adcs.mode));
  }

  // AUTO actuator selection: wheels at 0% saturation -> REACTION_WHEELS.
  {
    ADCS adcs;
    HardwareConfig hw = makeTestHardwareConfig();
    adcs.configure(hw, glm::quat(1, 0, 0, 0));
    adcs.mode = PointingMode::DETUMBLE;
    adcs.detumbleActuator = DetumbleActuator::AUTO;
    adcs.ambientFieldWorld = glm::vec3(0, 0, 3e-5f);

    FSWInputs in = makeInputs(glm::vec3(0.3f, 0.0f, 0.0f), /*wheelsHealthy=*/true, /*wheelSpeedRadS=*/0.0f);
    adcs.step(in, 0.05f);

    CHECK(adcs.activeDetumbleActuator == DetumbleActuator::REACTION_WHEELS,
          "AUTO picks REACTION_WHEELS when wheel saturation is 0%% (active = %d)",
          static_cast<int>(adcs.activeDetumbleActuator));
  }

  // AUTO actuator selection: wheels above the saturation budget -> MAGNETORQUERS_BDOT.
  {
    ADCS adcs;
    HardwareConfig hw = makeTestHardwareConfig();
    adcs.configure(hw, glm::quat(1, 0, 0, 0));
    adcs.mode = PointingMode::DETUMBLE;
    adcs.detumbleActuator = DetumbleActuator::AUTO;
    adcs.ambientFieldWorld = glm::vec3(0, 0, 3e-5f);

    // hw.wheels[i].maxSpeedRadS == 628.0f (see makeTestHardwareConfig()) --
    // speed well above detumbleWheelSaturationBudget * maxSpeedRadS.
    float highSpeed = 0.9f * 628.0f;
    FSWInputs in = makeInputs(glm::vec3(0.3f, 0.0f, 0.0f), /*wheelsHealthy=*/true, highSpeed);
    adcs.step(in, 0.05f);

    CHECK(adcs.activeDetumbleActuator == DetumbleActuator::MAGNETORQUERS_BDOT,
          "AUTO picks MAGNETORQUERS_BDOT when wheel saturation exceeds the budget (active = %d)",
          static_cast<int>(adcs.activeDetumbleActuator));
  }

  TEST_MAIN_END();
}
