// Regression tests for FDIR's fault detection and mode-override behavior
// (src/fsw/FDIR.h/.cpp) -- see docs/ALGORITHMS.md "FDIR / Mode Manager" for the
// fault model these are checking. Drives FlightSoftware (not a bare ADCS)
// since fault-driven mode override is FlightSoftware::step()'s job now that
// FDIR is a peer of ADCS rather than nested inside it.
//
// FlightSoftware::step() samples sensors/applies actuator commands itself,
// against a real Satellite (see FlightSoftware.h's own header comment on
// why that's not a Satellite method) -- so faults are injected by
// manipulating the simulated hardware directly (a wheel's own
// healthFactor, the body's own angularVelocity) rather than by
// constructing synthetic per-sensor readings the way test_adcs_control.cpp/
// test_detumble.cpp do against bare ADCS.
#include "test_common.h"
#include "FlightSoftware.h"
#include "core/Satellite.h"
#include <rigidbody/PhysicsWorld.h>

namespace
{
constexpr float DT = 0.05f;

// One representative environment for every scenario below -- FDIR doesn't
// care about guidance geometry/ground-station targeting, only about the
// fault-relevant signals (wheel health, rate, attitude uncertainty,
// battery), so these values are arbitrary but fixed. Sun held well away
// from the star tracker's default -Z boresight so it isn't blinded outside
// the excess-rate scenario (which blinds it anyway by exceeding its own
// max slew rate -- a real, and realistic, side effect of that fault).
void stepOnce(FlightSoftware &fsw)
{
  fsw.step(DT, glm::dvec3(0.0, 0.0, 7.0e6), 2451545.0,
           glm::vec3(1.5e11f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 3e-5f), false);
}

// Bundles a PhysicsWorld + real Satellite (buildSatellite()'s actual
// flight-hardware geometry) + FlightSoftware configured against it -- the
// minimum needed to drive FlightSoftware::step(), which owns sensor
// sampling/actuator commanding via a real Satellite.
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
  // Healthy system stays NOMINAL; commanded mode passes through untouched.
  {
    TestRig rig;
    ADCS &adcs = rig.fsw.adcs;
    adcs.mode = PointingMode::TARGET;
    for (int i = 0; i < 100; i++)
      stepOnce(rig.fsw);
    CHECK(rig.fsw.fdir.state() == FdirState::NOMINAL && adcs.effectiveMode == PointingMode::TARGET &&
              rig.fsw.systemMode() == SystemMode::NOMINAL,
          "Healthy system stays NOMINAL, effectiveMode == commanded TARGET, systemMode == NOMINAL");
  }

  // Losing 2 of 4 wheels (below minHealthyWheels) trips SAFE_HOLD and
  // overrides to SUN_POINTING, without touching the commanded mode.
  {
    TestRig rig;
    ADCS &adcs = rig.fsw.adcs;
    adcs.mode = PointingMode::TARGET;
    rig.sat.wheels[2]->healthFactor = 0.0f;
    rig.sat.wheels[3]->healthFactor = 0.0f;
    for (int i = 0; i < 100; i++)
      stepOnce(rig.fsw);
    CHECK(rig.fsw.fdir.state() == FdirState::SAFE_HOLD &&
              (rig.fsw.fdir.activeFaults() & FDIR_FAULT_WHEEL_AUTHORITY_LOST) &&
              adcs.mode == PointingMode::TARGET &&
              adcs.effectiveMode == PointingMode::SUN_POINTING &&
              rig.fsw.systemMode() == SystemMode::SAFE,
          "2 dead wheels -> SAFE_HOLD, commanded stays TARGET, effectiveMode -> SUN_POINTING, systemMode == SAFE");

    rig.fsw.fdir.clearLatchedFaults();
    CHECK(rig.fsw.fdir.state() == FdirState::NOMINAL && rig.fsw.systemMode() == SystemMode::NOMINAL,
          "clearLatchedFaults() immediately returns to NOMINAL");
  }

  // Excess rate wins over other faults and forces DETUMBLE.
  {
    TestRig rig;
    ADCS &adcs = rig.fsw.adcs;
    adcs.mode = PointingMode::TARGET;
    rig.sat.body->angularVelocity = glm::vec3(3.0f, 0.0f, 0.0f); // above excessRateRadS default (2.0)
    for (int i = 0; i < 10; i++)
      stepOnce(rig.fsw);
    CHECK(adcs.effectiveMode == PointingMode::DETUMBLE && rig.fsw.systemMode() == SystemMode::SAFE,
          "Excess rate -> effectiveMode forced to DETUMBLE, systemMode == SAFE");
  }

  // Autonomy disabled: fault still detected/latched but never overrides
  // effectiveMode -- the ground-inhibit switch actually inhibits.
  {
    TestRig rig;
    ADCS &adcs = rig.fsw.adcs;
    adcs.mode = PointingMode::TARGET;
    rig.fsw.fdir.enabled = false;
    rig.sat.wheels[0]->healthFactor = 0.0f;
    rig.sat.wheels[1]->healthFactor = 0.0f;
    rig.sat.wheels[2]->healthFactor = 0.0f;
    for (int i = 0; i < 100; i++)
      stepOnce(rig.fsw);
    CHECK((rig.fsw.fdir.activeFaults() & FDIR_FAULT_WHEEL_AUTHORITY_LOST) && adcs.effectiveMode == PointingMode::TARGET &&
              rig.fsw.systemMode() == SystemMode::NOMINAL,
          "Autonomy disabled: fault still latched but effectiveMode/systemMode unchanged");
  }

  TEST_MAIN_END();
}
