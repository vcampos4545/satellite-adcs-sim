#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

class PIDController
{
public:
  float Kp = 0.001f;
  float Ki = 0.0001f;
  float Kd = 0.002f;

  glm::vec3 integralError{0.0f};
  float maxIntegral = 0.1f;

public:
  PIDController() = default;

  void autoTune(glm::mat3 inertiaTensor,
                float settlingTime,
                float dampingRatio);

  glm::vec3 computeControlTorque(
      glm::quat targetAttitude,
      const glm::quat &q_current,
      const glm::vec3 &omega_current,
      float dt);

  void reset();
};

// Attitude regulator via LQR state feedback (u = -K x).
// Per-axis double-integrator model: theta_dot = omega, omega_dot = u / I_axis
// (small-angle attitude error, principal-axis inertia, cross-axis coupling ignored).
// The continuous-time algebraic Riccati equation for this 2-state/1-input system
// has a closed form, so no numerical Riccati solver is needed.
class LQRController
{
public:
  // LQR weights: state cost on [attitude error, angular rate], control cost on
  // torque. These are derived by autoTune() from settling time / damping ratio
  // (not hand-picked), so the resulting gains stay in the same physical range
  // as the actuators regardless of plant inertia -- exposed here for inspection.
  glm::vec3 Q_att{0.0f};
  glm::vec3 Q_rate{0.0f};
  glm::vec3 R{1.0f};

  // Gains solved by autoTune (unsaturated law: torque = -Kp * error + Kd * omega)
  glm::vec3 Kp{0.0f};
  glm::vec3 Kd{0.0f};

  // Commanded-rate saturation (rad/s), same role as CascadedController::omega_max:
  // bounds how fast a large attitude error is allowed to demand the body turn.
  float omega_max = 0.5f;

public:
  LQRController() = default;

  // Solve the per-axis CARE for gains that hit the given settling time / damping
  // ratio (same parameterization as PIDController::autoTune), so LQR's control
  // authority is tied to the physical response you actually want rather than an
  // arbitrary, unscaled Q/R pair that can vastly exceed actuator torque limits.
  void autoTune(glm::mat3 inertiaTensor,
                float settlingTime = 5.0f,
                float dampingRatio = 1.0f,
                float omega_max = 0.5f);

  glm::vec3 computeControlTorque(
      glm::quat targetAttitude,
      const glm::quat &q_current,
      const glm::vec3 &omega_current,
      float dt);

  // No persistent state, kept for interface parity with PIDController
  void reset() {}
};

// Cascaded attitude controller: outer loop maps attitude error -> rate command
// (with rate saturation for large slews), inner loop maps rate error -> torque.
class CascadedController
{
public:
  float settlingTime = 5.0f; // seconds
  float dampingRatio = 1.0f; // 1.0 = critically damped
  float omega_max = 0.5f;    // rad/s
  glm::mat3 inertiaTensor{1.0f};

public:
  CascadedController() = default;

  void autoTune(glm::mat3 inertiaTensor,
                float settlingTime = 5.0f,
                float dampingRatio = 1.0f,
                float omega_max = 0.5f);

  glm::vec3 computeControlTorque(
      glm::quat targetAttitude,
      const glm::quat &q_current,
      const glm::vec3 &omega_current,
      float dt);

  // No persistent state, kept for interface parity with PIDController
  void reset() {}
};
