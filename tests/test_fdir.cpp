// Regression tests for FDIR's fault detection and mode-override behavior
// (src/FDIR.h/.cpp) -- see docs/ALGORITHMS.md "FDIR / Mode Manager" for the
// fault model these are checking.
#include "test_common.h"
#include "ADCS.h"

int main()
{
  // Healthy system stays NOMINAL; commanded mode passes through untouched.
  {
    ADCS adcs;
    HardwareConfig hw = makeTestHardwareConfig();
    glm::quat trueAtt(1, 0, 0, 0);
    adcs.configure(hw, trueAtt);
    adcs.mode = PointingMode::TARGET;
    adcs.target = glm::vec3(0, 0, 5.0f);
    adcs.ambientFieldWorld = glm::vec3(0, 0, 3e-5f);
    float dt = 0.05f;
    for (int i = 0; i < 100; i++)
    {
      FSWInputs in;
      in.imu = {glm::vec3(0.0f), glm::vec3(0.0f)};
      in.mag = {glm::vec3(0, 0, 3e-5f), true};
      in.star = {trueAtt, true};
      in.power = {1.0f, 8.4f};
      in.spacecraftPositionWorld = glm::vec3(0.0f);
      for (int w = 0; w < NUM_WHEELS; w++)
        in.wheelTelemetry[w] = {0.0f, true};
      adcs.step(in, dt);
    }
    CHECK(adcs.fdir.state() == FdirState::NOMINAL && adcs.effectiveMode == PointingMode::TARGET,
          "Healthy system stays NOMINAL, effectiveMode == commanded TARGET");
  }

  // Losing 2 of 4 wheels (below minHealthyWheels) trips SAFE_HOLD and
  // overrides to SUN_POINTING, without touching the commanded mode.
  {
    ADCS adcs;
    HardwareConfig hw = makeTestHardwareConfig();
    glm::quat trueAtt(1, 0, 0, 0);
    adcs.configure(hw, trueAtt);
    adcs.mode = PointingMode::TARGET;
    adcs.target = glm::vec3(0, 0, 5.0f);
    adcs.sunPosition = glm::vec3(3.0f, 0.0f, 0.0f);
    adcs.ambientFieldWorld = glm::vec3(0, 0, 3e-5f);
    float dt = 0.05f;
    for (int i = 0; i < 100; i++)
    {
      FSWInputs in;
      in.imu = {glm::vec3(0.0f), glm::vec3(0.0f)};
      in.mag = {glm::vec3(0, 0, 3e-5f), true};
      in.star = {trueAtt, true};
      in.power = {1.0f, 8.4f};
      in.spacecraftPositionWorld = glm::vec3(0.0f);
      in.wheelTelemetry[0] = {0.0f, true};
      in.wheelTelemetry[1] = {0.0f, true};
      in.wheelTelemetry[2] = {0.0f, false};
      in.wheelTelemetry[3] = {0.0f, false};
      adcs.step(in, dt);
    }
    CHECK(adcs.fdir.state() == FdirState::SAFE_HOLD &&
              (adcs.fdir.activeFaults() & FDIR_FAULT_WHEEL_AUTHORITY_LOST) &&
              adcs.mode == PointingMode::TARGET &&
              adcs.effectiveMode == PointingMode::SUN_POINTING,
          "2 dead wheels -> SAFE_HOLD, commanded stays TARGET, effectiveMode -> SUN_POINTING");

    adcs.fdir.clearLatchedFaults();
    CHECK(adcs.fdir.state() == FdirState::NOMINAL, "clearLatchedFaults() immediately returns to NOMINAL");
  }

  // Excess rate wins over other faults and forces DETUMBLE.
  {
    ADCS adcs;
    HardwareConfig hw = makeTestHardwareConfig();
    glm::quat trueAtt(1, 0, 0, 0);
    adcs.configure(hw, trueAtt);
    adcs.mode = PointingMode::TARGET;
    adcs.target = glm::vec3(0, 0, 5.0f);
    adcs.ambientFieldWorld = glm::vec3(0, 0, 3e-5f);
    float dt = 0.05f;
    glm::vec3 highRate(3.0f, 0.0f, 0.0f); // above excessRateRadS default (2.0)
    for (int i = 0; i < 10; i++)
    {
      FSWInputs in;
      in.imu = {highRate, glm::vec3(0.0f)};
      in.mag = {glm::vec3(0, 0, 3e-5f), true};
      in.star = {trueAtt, true};
      in.power = {1.0f, 8.4f};
      in.spacecraftPositionWorld = glm::vec3(0.0f);
      for (int w = 0; w < NUM_WHEELS; w++)
        in.wheelTelemetry[w] = {0.0f, true};
      adcs.step(in, dt);
    }
    CHECK(adcs.effectiveMode == PointingMode::DETUMBLE, "Excess rate -> effectiveMode forced to DETUMBLE");
  }

  // Autonomy disabled: fault still detected/latched but never overrides
  // effectiveMode -- the ground-inhibit switch actually inhibits.
  {
    ADCS adcs;
    HardwareConfig hw = makeTestHardwareConfig();
    glm::quat trueAtt(1, 0, 0, 0);
    adcs.configure(hw, trueAtt);
    adcs.mode = PointingMode::TARGET;
    adcs.target = glm::vec3(0, 0, 5.0f);
    adcs.ambientFieldWorld = glm::vec3(0, 0, 3e-5f);
    adcs.fdir.enabled = false;
    float dt = 0.05f;
    for (int i = 0; i < 100; i++)
    {
      FSWInputs in;
      in.imu = {glm::vec3(0.0f), glm::vec3(0.0f)};
      in.mag = {glm::vec3(0, 0, 3e-5f), true};
      in.star = {trueAtt, true};
      in.power = {1.0f, 8.4f};
      in.spacecraftPositionWorld = glm::vec3(0.0f);
      in.wheelTelemetry[0] = {0.0f, false};
      in.wheelTelemetry[1] = {0.0f, false};
      in.wheelTelemetry[2] = {0.0f, false};
      in.wheelTelemetry[3] = {0.0f, true};
      adcs.step(in, dt);
    }
    CHECK((adcs.fdir.activeFaults() & FDIR_FAULT_WHEEL_AUTHORITY_LOST) && adcs.effectiveMode == PointingMode::TARGET,
          "Autonomy disabled: fault still latched but effectiveMode unchanged");
  }

  TEST_MAIN_END();
}
