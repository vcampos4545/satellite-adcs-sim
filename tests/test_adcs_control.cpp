// Regression tests for ADCS's core control/estimation loop (src/fsw/ADCS.h/.cpp,
// src/fsw/Controllers.h/.cpp) -- see docs/ALGORITHMS.md for the EKF, TRIAD,
// B-dot, and desaturation equations these are checking against. These are
// behavior-preservation tests: if a refactor changes the numbers here
// without an equally-updated doc explaining why, that's a signal something
// drifted, not just a threshold to bump.
#include "test_common.h"
#include "ADCS.h"
#include <glm/gtc/quaternion.hpp>
#include <cmath>

int main()
{
  // NADIR guidance targets -normalize(spacecraftPositionWorld) -- the
  // harness feeds this field the real orbital position (ECI, Earth's
  // center at the origin -- see docs/ALGORITHMS.md's "Orbital Mechanics"
  // section), so nadir is just the direction back toward it. Given a
  // known (LEO-radius-scale) position, the resulting targetAttitude must
  // rotate body +Z exactly onto the direction back to the origin.
  {
    ADCS adcs;
    HardwareConfig hw = makeTestHardwareConfig();
    adcs.configure(hw, glm::quat(1, 0, 0, 0));
    adcs.mode = PointingMode::NADIR;

    glm::vec3 spacecraftPos = glm::normalize(glm::vec3(1.0f, 0.5f, -0.3f)) * 6.871e6f; // 500km-altitude-scale LEO position
    glm::vec3 expectedNadirDir = -glm::normalize(spacecraftPos);

    FSWInputs in;
    in.imu = {glm::vec3(0.0f), glm::vec3(0.0f)};
    in.mag = {glm::vec3(0, 0, 3e-5f), true};
    in.star = {glm::quat(1, 0, 0, 0), true};
    in.power = {1.0f, 8.4f};
    in.spacecraftPositionWorld = spacecraftPos;
    for (int w = 0; w < NUM_WHEELS; w++)
      in.wheelTelemetry[w] = {0.0f, true};

    adcs.step(in, 0.05f);

    glm::vec3 achievedDir = adcs.targetAttitude * glm::vec3(0, 0, 1);
    float angleErrDeg = glm::degrees(std::acos(glm::clamp(glm::dot(achievedDir, expectedNadirDir), -1.0f, 1.0f)));
    CHECK(angleErrDeg < 0.01f,
          "NADIR guidance: targetAttitude rotates body +Z toward -spacecraftPositionWorld (angle err = %.4f deg)",
          angleErrDeg);
  }

  // B-dot detumble: body rate trends down under the B-dot law against a
  // representative LEO field. A ~30uT field gives the magnetorquers very
  // little torque authority against even this small bus, so this checks
  // direction (a real downward trend), not full convergence within a
  // simulatable duration -- see docs/ALGORITHMS.md.
  {
    ADCS adcs;
    HardwareConfig hw = makeTestHardwareConfig();
    glm::quat trueAtt(1, 0, 0, 0);
    glm::vec3 trueRate(0.3f, -0.2f, 0.25f);
    float initialRateMag = glm::length(trueRate);
    adcs.configure(hw, trueAtt);
    adcs.mode = PointingMode::DETUMBLE;
    adcs.detumbleActuator = DetumbleActuator::MAGNETORQUERS_BDOT;

    glm::vec3 fieldWorld = glm::normalize(glm::vec3(0.3f, 0.1f, 0.9f)) * 3e-5f;
    adcs.ambientFieldWorld = fieldWorld;
    glm::mat3 invI = glm::inverse(hw.busInertiaTensor);
    float dt = 0.05f;
    for (int i = 0; i < 40000; i++) // 2000s simulated
    {
      trueAtt = glm::normalize(trueAtt * glm::quat(1.0f, 0.5f * trueRate * dt));
      glm::vec3 fieldBody = glm::inverse(trueAtt) * fieldWorld;

      FSWInputs in;
      in.imu = {trueRate, glm::vec3(0.0f)};
      in.mag = {fieldBody, true};
      in.star = {trueAtt, true};
      in.power = {1.0f, 8.4f};
      in.spacecraftPositionWorld = glm::vec3(0.0f);
      for (int w = 0; w < NUM_WHEELS; w++)
        in.wheelTelemetry[w] = {0.0f, true};

      FSWOutputs out = adcs.step(in, dt);

      glm::vec3 dipole(0.0f);
      for (int t = 0; t < NUM_TORQUERS; t++)
        dipole += out.torquerCommands[t].momentAm2 * hw.torquers[t].axisBody;
      glm::vec3 torqueBody = glm::cross(dipole, fieldBody);
      trueRate += invI * torqueBody * dt;
    }
    float finalRateMag = glm::length(trueRate);
    CHECK(finalRateMag < initialRateMag * 0.9f,
          "B-dot detumble: |rate| %.4f -> %.4f rad/s over 2000s (want a clear downward trend)",
          initialRateMag, finalRateMag);
  }

  // Cascaded controller converges to the target attitude.
  {
    ADCS adcs;
    HardwareConfig hw = makeTestHardwareConfig();
    glm::quat trueAtt = glm::angleAxis(glm::radians(60.0f), glm::normalize(glm::vec3(0.2f, 1.0f, 0.3f)));
    glm::vec3 trueRate(0.0f);
    adcs.configure(hw, trueAtt);
    adcs.controllerType = ControllerType::CASCADED;
    adcs.mode = PointingMode::TARGET;
    adcs.target = glm::vec3(0.0f, 0.0f, 5.0f);

    glm::mat3 invI = glm::inverse(hw.busInertiaTensor);
    float dt = 0.05f;
    for (int i = 0; i < 3000; i++)
    {
      trueAtt = glm::normalize(trueAtt * glm::quat(1.0f, 0.5f * trueRate * dt));

      FSWInputs in;
      in.imu = {trueRate, glm::vec3(0.0f)};
      in.mag = {glm::vec3(0, 0, 3e-5f), true};
      in.star = {trueAtt, true};
      in.power = {1.0f, 8.4f};
      in.spacecraftPositionWorld = glm::vec3(0.0f);
      for (int w = 0; w < NUM_WHEELS; w++)
        in.wheelTelemetry[w] = {0.0f, true};

      FSWOutputs out = adcs.step(in, dt);

      glm::vec3 torqueBody(0.0f);
      for (int w = 0; w < NUM_WHEELS; w++)
        torqueBody += out.wheelCommands[w].torqueNm * hw.wheels[w].spinAxisBody;
      // Reaction torque on the bus is -torqueBody (Newton's third law,
      // matches ReactionWheel's actual convention in the sim/engine).
      trueRate += invI * (-torqueBody) * dt;
    }
    glm::quat errQ = glm::inverse(trueAtt) * adcs.targetAttitude;
    if (errQ.w < 0.0f)
      errQ = -errQ;
    float errDeg = glm::degrees(2.0f * std::acos(glm::clamp(errQ.w, -1.0f, 1.0f)));
    CHECK(errDeg < 2.0f, "Cascaded pointing: final true error = %.3f deg (want < 2.0)", errDeg);
  }

  // EKF converges even through periodic star-tracker dropouts, falling
  // back to the sun+magnetometer TRIAD solve.
  {
    ADCS adcs;
    HardwareConfig hw = makeTestHardwareConfig();
    glm::quat trueAtt = glm::angleAxis(glm::radians(20.0f), glm::normalize(glm::vec3(1, 0, 1)));
    adcs.configure(hw, glm::quat(1, 0, 0, 0)); // deliberately wrong initial estimate
    adcs.mode = PointingMode::TARGET;
    adcs.target = glm::vec3(1.0f, 0.0f, 0.0f);
    adcs.sunPosition = glm::vec3(0.0f, 5.0f, 0.0f);

    glm::vec3 fieldWorld = glm::normalize(glm::vec3(0.3f, 0.1f, 0.9f)) * 3e-5f;
    adcs.ambientFieldWorld = fieldWorld;
    float dt = 0.05f;
    bool sawFallback = false;
    for (int i = 0; i < 400; i++)
    {
      bool starValid = (i % 10) != 0; // drop out 1 in 10 cycles
      glm::vec3 fieldBody = glm::inverse(trueAtt) * fieldWorld;
      glm::vec3 sunDirWorld = glm::normalize(adcs.sunPosition);
      glm::vec3 sunDirBody = glm::inverse(trueAtt) * sunDirWorld;

      FSWInputs in;
      in.imu = {glm::vec3(0.0f), glm::vec3(0.0f)};
      in.mag = {fieldBody, true};
      in.star = {trueAtt, starValid};
      in.sunSensor = {sunDirBody, true};
      in.power = {1.0f, 8.4f};
      in.spacecraftPositionWorld = glm::vec3(0.0f);
      for (int w = 0; w < NUM_WHEELS; w++)
        in.wheelTelemetry[w] = {0.0f, true};

      adcs.step(in, dt);
      if (!starValid && adcs.triadFallbackUsed)
        sawFallback = true;
    }
    glm::quat errQ = glm::inverse(adcs.estimatedAttitude) * trueAtt;
    if (errQ.w < 0.0f)
      errQ = -errQ;
    float errDeg = glm::degrees(2.0f * std::acos(glm::clamp(errQ.w, -1.0f, 1.0f)));
    CHECK(errDeg < 1.0f && sawFallback,
          "EKF convergence w/ dropout: final estimate error = %.4f deg (want < 1.0), triad fallback used = %s",
          errDeg, sawFallback ? "yes" : "no");
  }

  // Desaturation drains a saturated wheel's momentum without destabilizing
  // pointing -- direction and stability, not speed (see B-dot's comment
  // above; the same weak-field-vs-tiny-bus reasoning applies).
  {
    ADCS adcs;
    HardwareConfig hw = makeTestHardwareConfig();
    glm::quat trueAtt(1, 0, 0, 0);
    adcs.configure(hw, trueAtt);
    adcs.mode = PointingMode::TARGET;
    adcs.target = glm::vec3(0, 0, 5.0f);
    adcs.ambientFieldWorld = glm::vec3(0, 0, 3e-5f);

    std::array<float, NUM_WHEELS> wheelSpeed{};
    wheelSpeed[0] = 0.85f * hw.wheels[0].maxSpeedRadS; // only wheel 0 near saturation -- an even split
                                                        // across the pyramid cancels to ~zero net momentum

    glm::vec3 trueRate(0.0f);
    glm::mat3 invI = glm::inverse(hw.busInertiaTensor);
    float dt = 0.05f;
    float maxErrDeg = 0.0f;
    for (int i = 0; i < 20000; i++) // 1000s simulated
    {
      trueAtt = glm::normalize(trueAtt * glm::quat(1.0f, 0.5f * trueRate * dt));

      FSWInputs in;
      in.imu = {trueRate, glm::vec3(0.0f)};
      in.mag = {glm::vec3(0, 0, 3e-5f), true};
      in.star = {trueAtt, true};
      in.power = {1.0f, 8.4f};
      in.spacecraftPositionWorld = glm::vec3(0.0f);
      for (int w = 0; w < NUM_WHEELS; w++)
        in.wheelTelemetry[w] = {wheelSpeed[w], true};

      FSWOutputs out = adcs.step(in, dt);

      glm::vec3 wheelTorqueOnBus(0.0f);
      for (int w = 0; w < NUM_WHEELS; w++)
      {
        wheelSpeed[w] += (out.wheelCommands[w].torqueNm / hw.wheels[w].wheelInertia) * dt;
        wheelTorqueOnBus += -out.wheelCommands[w].torqueNm * hw.wheels[w].spinAxisBody;
      }
      glm::vec3 dipole(0.0f);
      for (int t = 0; t < NUM_TORQUERS; t++)
        dipole += out.torquerCommands[t].momentAm2 * hw.torquers[t].axisBody;
      glm::vec3 magTorque = glm::cross(dipole, glm::vec3(0, 0, 3e-5f));

      trueRate += invI * (wheelTorqueOnBus + magTorque) * dt;

      if (i > 500)
      {
        glm::quat errQ = glm::inverse(trueAtt) * adcs.targetAttitude;
        if (errQ.w < 0.0f)
          errQ = -errQ;
        float errDeg = glm::degrees(2.0f * std::acos(glm::clamp(errQ.w, -1.0f, 1.0f)));
        maxErrDeg = std::max(maxErrDeg, errDeg);
      }
    }
    float finalMaxSat = 0.0f;
    for (int w = 0; w < NUM_WHEELS; w++)
      finalMaxSat = std::max(finalMaxSat, std::abs(wheelSpeed[w]) / hw.wheels[w].maxSpeedRadS);
    CHECK(finalMaxSat < 0.85f && maxErrDeg < 5.0f,
          "Desaturation: final max wheel sat = %.4f (want < 0.85, started at 0.85), max pointing err after settle = %.3f deg (want < 5.0)",
          finalMaxSat, maxErrDeg);
  }

  // TRIAD fallback must refuse when the sun reference isn't actually
  // available (e.g. the harness gates SunSensor::Reading.valid to false
  // during eclipse -- see satellite_adcs_sim.cpp's own comment on this,
  // since SunSensor itself has no eclipse model). computeTriadFallback
  // requires in.sunSensor.valid; with the star tracker also down (the
  // scenario TRIAD fallback exists for in the first place), ADCS must not
  // silently claim a TRIAD correction it can't actually perform.
  {
    ADCS adcs;
    HardwareConfig hw = makeTestHardwareConfig();
    adcs.configure(hw, glm::quat(1, 0, 0, 0));
    adcs.mode = PointingMode::TARGET;
    adcs.target = glm::vec3(1.0f, 0.0f, 0.0f);
    adcs.ambientFieldWorld = glm::vec3(0, 0, 3e-5f);

    FSWInputs in;
    in.imu = {glm::vec3(0.0f), glm::vec3(0.0f)};
    in.mag = {glm::vec3(0, 0, 3e-5f), true};
    in.star = {glm::quat(1, 0, 0, 0), false};    // star tracker down
    in.sunSensor = {glm::vec3(0, 1, 0), false};  // sun reference unavailable (eclipse)
    in.power = {1.0f, 8.4f};
    in.spacecraftPositionWorld = glm::vec3(0.0f);
    for (int w = 0; w < NUM_WHEELS; w++)
      in.wheelTelemetry[w] = {0.0f, true};

    adcs.step(in, 0.05f);

    CHECK(!adcs.triadFallbackUsed,
          "TRIAD fallback must not fire with an invalid sun sensor reading (eclipse), even with the star tracker also down");
  }

  TEST_MAIN_END();
}
