// Regression tests for FDIR's fault detection and mode-override behavior
// (src/fsw/FDIR.h/.cpp) -- see docs/ALGORITHMS.md "FDIR / Mode Manager" for the
// fault model these are checking. Drives FlightSoftware (not a bare ADCS)
// since fault-driven mode override is FlightSoftware::step()'s job now that
// FDIR is a peer of ADCS rather than nested inside it.
#include "test_common.h"
#include "FlightSoftware.h"

int main()
{
  // Healthy system stays NOMINAL; commanded mode passes through untouched.
  {
    FlightSoftware fsw;
    ADCS &adcs = fsw.adcs;
    HardwareConfig hw = makeTestHardwareConfig();
    glm::quat trueAtt(1, 0, 0, 0);
    fsw.configure(hw, trueAtt);
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
      fsw.step(in, dt);
    }
    CHECK(fsw.fdir.state() == FdirState::NOMINAL && adcs.effectiveMode == PointingMode::TARGET &&
              fsw.systemMode() == SystemMode::NOMINAL,
          "Healthy system stays NOMINAL, effectiveMode == commanded TARGET, systemMode == NOMINAL");
  }

  // Losing 2 of 4 wheels (below minHealthyWheels) trips SAFE_HOLD and
  // overrides to SUN_POINTING, without touching the commanded mode.
  {
    FlightSoftware fsw;
    ADCS &adcs = fsw.adcs;
    HardwareConfig hw = makeTestHardwareConfig();
    glm::quat trueAtt(1, 0, 0, 0);
    fsw.configure(hw, trueAtt);
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
      fsw.step(in, dt);
    }
    CHECK(fsw.fdir.state() == FdirState::SAFE_HOLD &&
              (fsw.fdir.activeFaults() & FDIR_FAULT_WHEEL_AUTHORITY_LOST) &&
              adcs.mode == PointingMode::TARGET &&
              adcs.effectiveMode == PointingMode::SUN_POINTING &&
              fsw.systemMode() == SystemMode::SAFE,
          "2 dead wheels -> SAFE_HOLD, commanded stays TARGET, effectiveMode -> SUN_POINTING, systemMode == SAFE");

    fsw.fdir.clearLatchedFaults();
    CHECK(fsw.fdir.state() == FdirState::NOMINAL && fsw.systemMode() == SystemMode::NOMINAL,
          "clearLatchedFaults() immediately returns to NOMINAL");
  }

  // Excess rate wins over other faults and forces DETUMBLE.
  {
    FlightSoftware fsw;
    ADCS &adcs = fsw.adcs;
    HardwareConfig hw = makeTestHardwareConfig();
    glm::quat trueAtt(1, 0, 0, 0);
    fsw.configure(hw, trueAtt);
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
      fsw.step(in, dt);
    }
    CHECK(adcs.effectiveMode == PointingMode::DETUMBLE && fsw.systemMode() == SystemMode::SAFE,
          "Excess rate -> effectiveMode forced to DETUMBLE, systemMode == SAFE");
  }

  // Autonomy disabled: fault still detected/latched but never overrides
  // effectiveMode -- the ground-inhibit switch actually inhibits.
  {
    FlightSoftware fsw;
    ADCS &adcs = fsw.adcs;
    HardwareConfig hw = makeTestHardwareConfig();
    glm::quat trueAtt(1, 0, 0, 0);
    fsw.configure(hw, trueAtt);
    adcs.mode = PointingMode::TARGET;
    adcs.target = glm::vec3(0, 0, 5.0f);
    adcs.ambientFieldWorld = glm::vec3(0, 0, 3e-5f);
    fsw.fdir.enabled = false;
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
      fsw.step(in, dt);
    }
    CHECK((fsw.fdir.activeFaults() & FDIR_FAULT_WHEEL_AUTHORITY_LOST) && adcs.effectiveMode == PointingMode::TARGET &&
              fsw.systemMode() == SystemMode::NOMINAL,
          "Autonomy disabled: fault still latched but effectiveMode/systemMode unchanged");
  }

  TEST_MAIN_END();
}
