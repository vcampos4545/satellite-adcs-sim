// Regression tests for ADCS's target-loss guidance fallback (ADCS.h's
// `targetValid` field) -- see docs/ALGORITHMS.md's "Ground-station
// targeting"/"Target-loss fallback" sections. Not FDIR-related; lives here
// (not test_adcs_control.cpp) since it's the other FlightSoftware-era
// behavior change from the same refactor.
#include "test_common.h"
#include "ADCS.h"
#include <cmath>

namespace
{
// Runs one ADCS::step() in `mode` with `targetValid` and returns the
// resulting targetAttitude's body +Z direction in world space -- matches
// test_adcs_control.cpp's NADIR test style.
glm::vec3 achievedDirFor(PointingMode mode, bool targetValid, glm::vec3 spacecraftPos, glm::vec3 sunPos, glm::vec3 target)
{
  ADCS adcs;
  HardwareConfig hw = makeTestHardwareConfig();
  adcs.configure(hw, glm::quat(1, 0, 0, 0));
  adcs.mode = mode;
  adcs.target = target;
  adcs.sunPosition = sunPos;
  adcs.targetValid = targetValid;

  ImuSample imu{glm::vec3(0.0f), glm::vec3(0.0f)};
  MagSample mag{glm::vec3(0, 0, 3e-5f), true};
  StarTrackerSample star{glm::quat(1, 0, 0, 0), true};
  SunSensorSample sunSensor;
  PowerSample power{1.0f, 8.4f};
  std::array<WheelTelemetry, NUM_WHEELS> wheelTelemetry{};
  for (int w = 0; w < NUM_WHEELS; w++)
    wheelTelemetry[w] = {0.0f, true};

  adcs.step(imu, mag, star, sunSensor, power, wheelTelemetry, spacecraftPos, 0.05f);
  return adcs.targetAttitude * glm::vec3(0, 0, 1);
}

float angleBetweenDeg(glm::vec3 a, glm::vec3 b)
{
  return glm::degrees(std::acos(glm::clamp(glm::dot(a, b), -1.0f, 1.0f)));
}
} // namespace

int main()
{
  glm::vec3 spacecraftPos(1.0e6f, 2.0e6f, -0.5e6f);
  glm::vec3 sunPos = glm::normalize(glm::vec3(0.2f, -0.6f, 0.8f)) * 1.5e11f;
  glm::vec3 expectedSunDir = glm::normalize(sunPos - spacecraftPos);

  // TARGET/SLEW/FINE_POINTING with an invalid target all fall back to
  // exactly the direction SUN_POINTING itself would produce.
  glm::vec3 sunPointingDir = achievedDirFor(PointingMode::SUN_POINTING, true, spacecraftPos, sunPos, glm::vec3(0.0f));

  for (PointingMode mode : {PointingMode::TARGET, PointingMode::SLEW, PointingMode::FINE_POINTING})
  {
    glm::vec3 achieved = achievedDirFor(mode, false, spacecraftPos, sunPos, glm::vec3(5.0f, 5.0f, 5.0f));
    float errDeg = angleBetweenDeg(achieved, sunPointingDir);
    CHECK(errDeg < 0.01f,
          "targetValid=false falls back to SUN_POINTING's direction (mode=%d, angle err = %.4f deg)",
          static_cast<int>(mode), errDeg);
  }

  // REFLECT with an invalid target also falls back to pointing straight at
  // the sun (no target means no bisector to compute).
  {
    glm::vec3 achieved = achievedDirFor(PointingMode::REFLECT, false, spacecraftPos, sunPos, glm::vec3(-3.0f, 1.0f, 2.0f));
    float errDeg = angleBetweenDeg(achieved, expectedSunDir);
    CHECK(errDeg < 0.01f, "REFLECT with targetValid=false points straight at the sun (angle err = %.4f deg)", errDeg);
  }

  // Sanity check: with a valid target, TARGET does NOT fall back to
  // sun-pointing (guards against the fallback firing unconditionally).
  {
    glm::vec3 target(0.0f, 0.0f, 5.0e6f);
    glm::vec3 achieved = achievedDirFor(PointingMode::TARGET, true, spacecraftPos, sunPos, target);
    float errVsSun = angleBetweenDeg(achieved, sunPointingDir);
    CHECK(errVsSun > 1.0f, "targetValid=true does not fall back to SUN_POINTING's direction (angle vs sun-dir = %.2f deg)", errVsSun);
  }

  TEST_MAIN_END();
}
