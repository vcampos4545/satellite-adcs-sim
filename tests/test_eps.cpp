// Regression tests for the EPS models (SolarPanel/Battery, in
// spacecraft-dynamics-sim) and FDIR's low-battery fault -- see
// docs/ALGORITHMS.md "EPS" for the equations/assumptions these check.
//
// Unlike test_adcs_control.cpp/test_fdir.cpp, this file needs `rigidbody`
// (not just `fsw`) since SolarPanel::sample() takes a real RigidBody --
// it's a physics primitive, not FSW.
#include "test_common.h"
#include "FlightSoftware.h"
#include <rigidbody/RigidBody.h>
#include <rigidbody/power/SolarPanel.h>
#include <rigidbody/power/Battery.h>

int main()
{
  // SolarPanel cosine law.
  {
    RigidBody body(RigidBodyShape::BOX, glm::vec3(0.1f, 0.1f, 0.1f));
    body.orientation = glm::quat(1, 0, 0, 0);
    body.position = glm::vec3(0.0f);

    SolarPanel panel(glm::vec3(0, 0, 1), 0.01f, 0.28f); // +Z normal, 1361*0.01*0.28 = 3.8108W max

    auto rNormal = panel.sample(body, glm::vec3(0, 0, 5.0f), 1361.0f);
    float expectedMax = 1361.0f * 0.01f * 0.28f;
    CHECK(std::abs(rNormal.powerW - expectedMax) < 1e-3f && rNormal.incidenceAngleDeg < 0.1f,
          "Normal incidence: %.4f W (want %.4f), angle %.2f deg (want ~0)",
          rNormal.powerW, expectedMax, rNormal.incidenceAngleDeg);

    auto rEdge = panel.sample(body, glm::vec3(5.0f, 0, 0), 1361.0f);
    CHECK(rEdge.powerW < 1e-3f && std::abs(rEdge.incidenceAngleDeg - 90.0f) < 0.1f,
          "Edge-on: %.4f W (want ~0), angle %.2f deg (want ~90)", rEdge.powerW, rEdge.incidenceAngleDeg);

    auto rBehind = panel.sample(body, glm::vec3(0, 0, -5.0f), 1361.0f);
    CHECK(rBehind.powerW == 0.0f, "Sun behind panel: %.4f W (want exactly 0, clamped not negative)", rBehind.powerW);
  }

  // Battery charge/discharge/clamping/voltage.
  {
    Battery bat(10.0f /* Wh */, 6.0f, 8.4f, 0.5f); // 50% of 36000J = 18000J
    CHECK(std::abs(bat.stateOfCharge() - 0.5f) < 1e-4f, "Initial SOC: %.4f (want 0.5)", bat.stateOfCharge());

    bat.update(10.0f, 100.0f); // +10W for 100s = +1000J
    float expectedJ = 18000.0f + 1000.0f;
    CHECK(std::abs(bat.energyJ - expectedJ) < 1.0f, "Charge +1000J: energyJ=%.1f (want %.1f)", bat.energyJ, expectedJ);

    bat.update(-1000.0f, 1000.0f); // way past empty -- must clamp to 0
    CHECK(bat.energyJ == 0.0f && bat.isDepleted(), "Over-discharge clamps to 0, isDepleted()=true");

    bat.update(1000.0f, 1000.0f); // way past full -- must clamp to capacity
    CHECK(bat.energyJ == bat.capacityJ, "Overcharge clamps to capacity");

    CHECK(std::abs(bat.voltage() - 8.4f) < 1e-4f, "Full battery voltage = %.3f (want 8.4)", bat.voltage());
  }

  // FDIR: low battery forces SUN_POINTING without touching the commanded mode.
  {
    FlightSoftware fsw;
    ADCS &adcs = fsw.adcs;
    FakeFlightSoftwareHAL hal;
    HardwareConfig hw = makeTestHardwareConfig();
    glm::quat trueAtt(1, 0, 0, 0);
    float dt = 0.05f;
    fsw.configure(hw, makeTestSchedule(dt), trueAtt, hal);
    adcs.mode = PointingMode::TARGET;
    adcs.target = glm::vec3(0, 0, 5.0f);
    adcs.ambientFieldWorld = glm::vec3(0, 0, 3e-5f);
    hal.imu = {glm::vec3(0.0f), glm::vec3(0.0f)};
    hal.mag = {glm::vec3(0, 0, 3e-5f), true};
    hal.star = {trueAtt, true};
    hal.power = {0.1f, 6.5f}; // 10% SOC, below default 20% trigger
    hal.navPosition = glm::vec3(0.0f);
    for (int w = 0; w < NUM_WHEELS; w++)
      hal.wheelTelemetry[w] = {0.0f, true};
    for (int i = 0; i < 50; i++)
      fsw.step(dt);
    CHECK(fsw.fdir.state() == FdirState::SAFE_HOLD &&
              (fsw.fdir.activeFaults() & FDIR_FAULT_LOW_BATTERY) &&
              adcs.mode == PointingMode::TARGET &&
              adcs.effectiveMode == PointingMode::SUN_POINTING &&
              std::abs(adcs.batterySoc - 0.1f) < 1e-4f &&
              fsw.systemMode() == SystemMode::SAFE,
          "Low battery (10%% SOC) -> SAFE_HOLD, effectiveMode -> SUN_POINTING, adcs.batterySoc mirrors input, systemMode == SAFE");
  }

  // Healthy battery -- no fault.
  {
    FlightSoftware fsw;
    ADCS &adcs = fsw.adcs;
    FakeFlightSoftwareHAL hal;
    HardwareConfig hw = makeTestHardwareConfig();
    glm::quat trueAtt(1, 0, 0, 0);
    float dt = 0.05f;
    fsw.configure(hw, makeTestSchedule(dt), trueAtt, hal);
    adcs.mode = PointingMode::TARGET;
    adcs.target = glm::vec3(0, 0, 5.0f);
    adcs.ambientFieldWorld = glm::vec3(0, 0, 3e-5f);
    hal.imu = {glm::vec3(0.0f), glm::vec3(0.0f)};
    hal.mag = {glm::vec3(0, 0, 3e-5f), true};
    hal.star = {trueAtt, true};
    hal.power = {0.8f, 8.0f};
    hal.navPosition = glm::vec3(0.0f);
    for (int w = 0; w < NUM_WHEELS; w++)
      hal.wheelTelemetry[w] = {0.0f, true};
    for (int i = 0; i < 50; i++)
      fsw.step(dt);
    CHECK(fsw.fdir.state() == FdirState::NOMINAL && adcs.effectiveMode == PointingMode::TARGET &&
              fsw.systemMode() == SystemMode::NOMINAL,
          "Healthy battery (80%% SOC) -> stays NOMINAL");
  }

  TEST_MAIN_END();
}
