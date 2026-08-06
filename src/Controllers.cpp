#include "Controllers.h"
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/common.hpp>
#include <cmath>
#include <algorithm>

void PIDController::autoTune(glm::mat3 inertiaTensor,
                             float settlingTime,
                             float dampingRatio)
{
  float I =
      (inertiaTensor[0][0] +
       inertiaTensor[1][1] +
       inertiaTensor[2][2]) /
      3.0f;

  float omega_n = 4.0f / (dampingRatio * settlingTime);

  Kp = I * omega_n * omega_n;
  Kd = 2.0f * dampingRatio * I * omega_n;
  Ki = Kp * 0.01f;

  if (Ki > 0.0f)
  {
    maxIntegral = (Kp / Ki) * 0.5f;
  }
}

glm::vec3 PIDController::computeControlTorque(
    glm::quat targetAttitude,
    const glm::quat &q_current,
    const glm::vec3 &omega_current,
    float dt)
{
  // Quaternion attitude error in BODY frame
  glm::quat q_error = glm::inverse(q_current) * targetAttitude;

  // Shortest rotation
  if (q_error.w < 0.0f)
  {
    q_error = -q_error;
  }

  // Axis-angle approximation
  glm::vec3 attitudeError(q_error.x, q_error.y, q_error.z);
  attitudeError *= 2.0f;

  // Integral update (anti-windup)
  integralError += attitudeError * dt;
  integralError = glm::clamp(
      integralError,
      -glm::vec3(maxIntegral),
      glm::vec3(maxIntegral));

  // Derivative (target angular velocity = 0)
  glm::vec3 derivativeError = -omega_current;

  glm::vec3 torque =
      Kp * attitudeError +
      Ki * integralError +
      Kd * derivativeError;

  return -torque;
}

void PIDController::reset()
{
  integralError = glm::vec3(0.0f);
}

void CascadedController::autoTune(glm::mat3 inertiaTensor_,
                                  float settlingTime_,
                                  float dampingRatio_,
                                  float omega_max_)
{
  inertiaTensor = inertiaTensor_;
  settlingTime = settlingTime_;
  dampingRatio = dampingRatio_;
  omega_max = omega_max_;
}

glm::vec3 CascadedController::computeControlTorque(
    glm::quat targetAttitude,
    const glm::quat &q_current,
    const glm::vec3 &omega_current,
    float dt)
{
  // Quaternion error (body frame), q_err rotates CURRENT -> TARGET
  glm::quat q_error = glm::inverse(q_current) * targetAttitude;

  // Shortest rotation
  if (q_error.w < 0.0f)
    q_error = -q_error;

  // Axis-angle approximation
  glm::vec3 attitudeError = 2.0f * glm::vec3(q_error.x, q_error.y, q_error.z);

  float omega_n = 4.0f / (dampingRatio * settlingTime);

  // OUTER LOOP: attitude error -> rate command. Positive gain: a positive
  // attitudeError means "rotate positively to reach target", so the
  // commanded rate should point the same way.
  float k_q = omega_n;
  glm::vec3 omega_cmd = k_q * attitudeError;

  // Rate saturation (limits slew rate for large attitude errors)
  float omega_cmd_mag = glm::length(omega_cmd);
  if (omega_cmd_mag > omega_max && omega_cmd_mag > 0.0f)
  {
    omega_cmd *= (omega_max / omega_cmd_mag);
  }

  // INNER LOOP: rate tracking -> torque. Positive gain here (no leading
  // negation): this returns the *physical* stabilizing torque directly
  // (+Kp*e - Kd*omega in expanded form), which -- same as
  // PIDController::computeControlTorque's final `return -torque` -- is
  // the opposite sign of what allocateActuators()/ReactionWheel::apply()
  // actually apply to the body (Newton's-third-law reaction convention).
  // The previous version had both this sign and the outer loop's sign
  // backwards at once: the attitude term came out correctly signed by
  // coincidence, but the rate/damping term ended up as +Kd*omega instead
  // of -Kd*omega -- active anti-damping, not stabilization. Confirmed via
  // a 60s headless run: the old code left attitude error oscillating
  // between 30-55 deg indefinitely with body rate pinned near omega_max,
  // never settling; this fix converges the same scenario to <2 deg.
  float omega_rate = 5.0f * omega_n;
  glm::vec3 rate_error = omega_current - omega_cmd;
  glm::vec3 tau = (inertiaTensor * rate_error) * omega_rate;

  return tau;
}

// class MPCController
// {

//   glm::vec3 computeControlTorque() { return {0, 0, 0}; }
// };

void LQRController::autoTune(glm::mat3 inertiaTensor,
                             float settlingTime,
                             float dampingRatio,
                             float omega_max_)
{
  omega_max = omega_max_;

  // Solve the CARE per-axis for A = [[0,1],[0,0]], B = [[0],[b]], b = 1/I_axis
  // Closed form (see header comment):
  //   p12 = sqrt(q1*r) / b
  //   p22 = sqrt(r * (2*p12 + q2)) / b
  //   K   = [b*p12/r, b*p22/r]
  for (int i = 0; i < 3; i++)
  {
    float I = inertiaTensor[i][i];
    float b = (I > 1e-9f) ? (1.0f / I) : 0.0f;

    if (b <= 0.0f)
    {
      Q_att[i] = 0.0f;
      Q_rate[i] = 0.0f;
      R[i] = 1.0f;
      Kp[i] = 0.0f;
      Kd[i] = 0.0f;
      continue;
    }

    // Target the same closed-loop pole location PIDController::autoTune uses, so
    // the resulting gains land in the same physical (torque) range the actuators
    // can actually deliver, instead of being an arbitrary, unscaled Q/R pair.
    float omega_n = 4.0f / (dampingRatio * settlingTime);
    float KpTarget = I * omega_n * omega_n;
    float KdTarget = 2.0f * dampingRatio * I * omega_n;

    // Back-solve the LQR weights (fixing r=1) that produce this target. Kp is
    // independent of I for this system (a property of the double-integrator
    // CARE), so Q_att = Kp^2 exactly. Q_rate is clamped at 0 because a diagonal
    // Q requires Q >= 0: pure position weighting alone already gives ~0.707
    // damping, so a *lower* damping ratio than that isn't reachable this way.
    float r = 1.0f;
    float q1 = KpTarget * KpTarget;
    float q2 = std::max(KdTarget * KdTarget - 2.0f * KpTarget * I, 0.0f);

    Q_att[i] = q1;
    Q_rate[i] = q2;
    R[i] = r;

    float p12 = std::sqrt(q1 * r) / b;
    float p22 = std::sqrt(r * (2.0f * p12 + q2)) / b;

    Kp[i] = b * p12 / r;
    Kd[i] = b * p22 / r;
  }
}

glm::vec3 LQRController::computeControlTorque(
    glm::quat targetAttitude,
    const glm::quat &q_current,
    const glm::vec3 &omega_current,
    float dt)
{
  // Quaternion attitude error in BODY frame
  glm::quat q_error = glm::inverse(q_current) * targetAttitude;

  // Shortest rotation
  if (q_error.w < 0.0f)
  {
    q_error = -q_error;
  }

  // Axis-angle approximation
  glm::vec3 attitudeError(q_error.x, q_error.y, q_error.z);
  attitudeError *= 2.0f;

  // Rewrite u = -Kp*e + Kd*omega as u = Kd*(omega - omega_cmd), omega_cmd =
  // (Kp/Kd)*e. This reproduces the unsaturated LQR law exactly, but exposes an
  // intermediate rate command that can be saturated -- the same protection
  // CascadedController::omega_max gives, so a large step-changed target can't
  // demand more torque than the rate loop is willing to track.
  glm::vec3 omega_cmd{0.0f};
  for (int i = 0; i < 3; i++)
    omega_cmd[i] = (Kd[i] > 1e-9f) ? (Kp[i] / Kd[i]) * attitudeError[i] : 0.0f;

  float omega_cmd_mag = glm::length(omega_cmd);
  if (omega_cmd_mag > omega_max && omega_cmd_mag > 0.0f)
  {
    omega_cmd *= (omega_max / omega_cmd_mag);
  }

  glm::vec3 torque = Kd * (omega_current - omega_cmd);

  return torque;
}