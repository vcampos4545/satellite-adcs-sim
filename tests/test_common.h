#pragma once
#include "FlightTypes.h"
#include "FlightSoftwareHAL.h"
#include <cstdio>
#include <glm/gtc/constants.hpp>

// Minimal, dependency-free test harness -- this project doesn't pull in
// GoogleTest/Catch2 for the same "keep dependencies to what's actually
// needed" reasoning the engine repo's own CMakeLists.txt follows. Each
// test file is its own small `main()`; CHECK() reports and keeps going
// (one failure doesn't hide the rest of the file's results), and the
// file's `main()` returns the accumulated failure count so `ctest` reads
// a real pass/fail exit code, not just "did it crash."
//
// See docs/ALGORITHMS.md for the equations/assumptions these tests are
// checking against, and docs/TESTING.md for when a test belongs here vs.
// being a one-off scratch check.
inline int g_testFailures = 0;

#define CHECK(cond, ...)                                  \
  do                                                       \
  {                                                        \
    bool _pass = (cond);                                   \
    std::printf("[%s] ", _pass ? "PASS" : "FAIL");          \
    std::printf(__VA_ARGS__);                               \
    std::printf("\n");                                       \
    if (!_pass)                                                \
      g_testFailures++;                                         \
  } while (0)

#define TEST_MAIN_END()                                                        \
  std::printf("\n%s\n", g_testFailures == 0 ? "ALL TESTS PASSED" : "SOME TESTS FAILED"); \
  return g_testFailures == 0 ? 0 : 1;

// A representative 4-wheel pyramid + 3-axis torquer HardwareConfig, the
// same shape buildCubesatPyramid() in satellite_adcs_sim.cpp builds --
// duplicated here (not shared with the harness) deliberately: ADCS is
// hardware-abstracted specifically so it never needs the harness/sim to
// exist at all, and a test suite that had to link the GUI harness to
// build a HardwareConfig would quietly defeat that.
inline HardwareConfig makeTestHardwareConfig()
{
  HardwareConfig hw;
  glm::vec3 axes[4] = {
      glm::normalize(glm::vec3(1, 1, 1)),
      glm::normalize(glm::vec3(1, -1, -1)),
      glm::normalize(glm::vec3(-1, 1, -1)),
      glm::normalize(glm::vec3(-1, -1, 1)),
  };
  for (int i = 0; i < NUM_WHEELS; i++)
    hw.wheels[i] = {axes[i], 0.001f, 628.0f, 1e-6f}; // matches buildCubesatPyramid()'s wheel spec
  glm::vec3 torquerAxes[3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  for (int i = 0; i < NUM_TORQUERS; i++)
    hw.torquers[i] = {torquerAxes[i], 0.2f};
  hw.busInertiaTensor = glm::mat3(0.002f, 0, 0, 0, 0.002f, 0, 0, 0, 0.002f);
  return hw;
}

// Every period equal to `dt` -- every sensor + the ADCS task fires on every
// single FlightSoftware::step(dt) call, so a test loop that calls step()
// once per iteration gets exactly one full FSW cycle per call, matching the
// cadence tests already expect.
inline FswSchedule makeTestSchedule(float dt)
{
  FswSchedule s;
  s.imuPeriodS = s.magPeriodS = s.starTrackerPeriodS = s.sunSensorPeriodS =
      s.powerPeriodS = s.navPeriodS = s.adcsPeriodS = dt;
  return s;
}

// A settable fake HAL for tests: set the public sample fields directly
// (mirroring how test files already hand-build FSWInputs), call
// FlightSoftware::step(dt), then read lastWheelTorqueNm/
// lastTorquerMomentAm2 or check FlightSoftware/ADCS/FDIR state.
struct FakeFlightSoftwareHAL : public FlightSoftwareHAL
{
  ImuSample imu;
  MagSample mag;
  StarTrackerSample star;
  SunSensorSample sunSensor;
  PowerSample power;
  std::array<WheelTelemetry, NUM_WHEELS> wheelTelemetry{};
  glm::vec3 navPosition{0.0f};
  std::array<float, NUM_WHEELS> lastWheelTorqueNm{};
  std::array<float, NUM_TORQUERS> lastTorquerMomentAm2{};

  ImuSample readImu(float) override { return imu; }
  MagSample readMag(float) override { return mag; }
  StarTrackerSample readStarTracker() override { return star; }
  SunSensorSample readSunSensor() override { return sunSensor; }
  PowerSample readPower() override { return power; }
  std::array<WheelTelemetry, NUM_WHEELS> readWheelTelemetry() override { return wheelTelemetry; }
  glm::vec3 readNavPosition() override { return navPosition; }
  void commandWheelTorque(int i, float t) override { lastWheelTorqueNm[i] = t; }
  void commandTorquerMoment(int i, float m) override { lastTorquerMomentAm2[i] = m; }
};
