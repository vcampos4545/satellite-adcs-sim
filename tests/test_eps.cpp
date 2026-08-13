// Regression tests for the EPS models (SolarPanel/Battery, in
// spacecraft-dynamics-sim) and FDIR's low-battery fault -- see
// docs/ALGORITHMS.md "EPS" for the equations/assumptions these check.
//
// Unlike test_adcs_control.cpp/test_detumble.cpp, this file needs
// `rigidbody` (not just `fsw`) since SolarPanel::sample() takes a real
// RigidBody -- it's a physics primitive, not FSW. The FDIR/battery
// scenarios below additionally need a real Satellite (see test_fdir.cpp's
// own header comment on why FlightSoftware::step() requires one now).
#include "test_common.h"
#include "FlightSoftware.h"
#include "core/Satellite.h"
#include <rigidbody/RigidBody.h>
#include <rigidbody/PhysicsWorld.h>
#include <rigidbody/power/SolarPanel.h>
#include <rigidbody/power/Battery.h>

namespace
{
constexpr float DT = 0.05f;

void stepOnce(FlightSoftware &fsw)
{
  fsw.step(DT, glm::dvec3(0.0, 0.0, 7.0e6), 2451545.0,
           glm::vec3(1.5e11f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 3e-5f), false);
}

// See test_fdir.cpp's own TestRig -- same shape, duplicated here rather
// than shared since these are two independent, self-contained test
// executables (matching this project's "one binary per module/feature"
// test convention).
struct TestRig
{
  PhysicsWorld world;
  Satellite sat;
  FlightSoftware fsw;

  TestRig() : sat(buildSatellite(world)), fsw(sat)
  {
    fsw.configure(sat.hwConfig, glm::quat(1, 0, 0, 0));
  }
};
} // namespace

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
    TestRig rig;
    ADCS &adcs = rig.fsw.adcs;
    adcs.mode = PointingMode::TARGET;
    rig.sat.battery = Battery(40.0f, 6.0f, 8.4f, 0.1f); // 10% SOC, below default 20% trigger
    for (int i = 0; i < 50; i++)
      stepOnce(rig.fsw);
    CHECK(rig.fsw.fdir.state() == FdirState::SAFE_HOLD &&
              (rig.fsw.fdir.activeFaults() & FDIR_FAULT_LOW_BATTERY) &&
              adcs.mode == PointingMode::TARGET &&
              adcs.effectiveMode == PointingMode::SUN_POINTING &&
              adcs.batterySoc < 0.2f &&
              rig.fsw.systemMode() == SystemMode::SAFE,
          "Low battery (10%% SOC) -> SAFE_HOLD, effectiveMode -> SUN_POINTING, adcs.batterySoc mirrors battery, systemMode == SAFE");
  }

  // Healthy battery -- no fault.
  {
    TestRig rig;
    ADCS &adcs = rig.fsw.adcs;
    adcs.mode = PointingMode::TARGET;
    rig.sat.battery = Battery(40.0f, 6.0f, 8.4f, 0.8f);
    for (int i = 0; i < 50; i++)
      stepOnce(rig.fsw);
    CHECK(rig.fsw.fdir.state() == FdirState::NOMINAL && adcs.effectiveMode == PointingMode::TARGET &&
              rig.fsw.systemMode() == SystemMode::NOMINAL,
          "Healthy battery (80%% SOC) -> stays NOMINAL");
  }

  TEST_MAIN_END();
}
