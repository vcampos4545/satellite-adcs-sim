#include "ADCS.h"
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/constants.hpp>
#include <cmath>
#include <algorithm>

namespace
{
struct ModeTuning
{
  float settlingTime;
  float dampingRatio;
  float omega_max;
};

// Per-mode gain/rate-limit presets. SLEW trades precision for speed (short
// settling time, high rate cap); FINE_POINTING trades speed for precision
// (long settling time, overdamped, tight rate cap). Everything else uses
// the same moderate tuning the original single-mode cubesat used.
ModeTuning tuningForMode(PointingMode mode)
{
  switch (mode)
  {
  case PointingMode::SLEW:
    return {2.0f, 0.9f, 1.0f};
  case PointingMode::FINE_POINTING:
    return {12.0f, 1.3f, 0.08f};
  case PointingMode::NADIR:
  case PointingMode::SUN_POINTING:
  case PointingMode::TARGET:
  case PointingMode::DETUMBLE:
  case PointingMode::REFLECT:
  default:
    return {5.0f, 1.0f, 0.5f};
  }
}

// S such that S*x == cross(v, x) for any x -- see propagateEstimator()'s
// F-matrix comment. Verified against glm::cross() in a headless test
// before use; glm::mat3's 9-scalar constructor is column-major, which is
// easy to get backwards.
glm::mat3 skewSymmetric(const glm::vec3 &v)
{
  return glm::mat3(0.0f, v.z, -v.y,
                   -v.z, 0.0f, v.x,
                    v.y, -v.x, 0.0f);
}

// Fixed, known gyro sensor spec used to seed/drive the EKF's covariance --
// mirrors this project's simulated IMU's own default noise figures (see
// rigidbody/sensors/IMU.h in the engine repo), since a real system's EKF
// tuning uses the sensor's datasheet spec, not a live query against a
// sensor object it no longer has a pointer to.
constexpr float GYRO_NOISE_STD_RAD_S = 0.0008f;
constexpr float GYRO_BIAS_DRIFT_STD_RAD_S = 0.0003f;
constexpr float GYRO_BIAS_RANGE_RAD_S = 0.01f;
} // namespace

void ADCS::configure(const HardwareConfig &hw, const glm::quat &initialAttitude)
{
  hw_ = hw;

  estimatedAttitude = initialAttitude;
  gyroBiasEstimate = glm::vec3(0.0f);

  // EKF covariance priors. Attitude starts with a modest nonzero
  // uncertainty even when seeded from a known-good initial attitude --
  // starting at literally zero would make the very first Kalman gain
  // computation divide by (numerically) nothing. Bias uncertainty is
  // seeded from the known gyro turn-on bias range above.
  float initialAttitudeUncertaintyRad = glm::radians(1.0f);
  covAA = glm::mat3(initialAttitudeUncertaintyRad * initialAttitudeUncertaintyRad);
  covAB = glm::mat3(0.0f);
  covBB = glm::mat3(GYRO_BIAS_RANGE_RAD_S * GYRO_BIAS_RANGE_RAD_S);

  gyroNoisePsd = GYRO_NOISE_STD_RAD_S * GYRO_NOISE_STD_RAD_S;
  gyroBiasWalkPsd = GYRO_BIAS_DRIFT_STD_RAD_S * GYRO_BIAS_DRIFT_STD_RAD_S;

  effectiveMode = mode; // fdir hasn't run yet -- no fault to override with
  lastTunedMode = effectiveMode;
  retuneForMode();

  // Detumble's rate-damping gain isn't part of the quaternion-error
  // controllers' tuning; derive it from bus inertia the same way autoTune
  // derives Kd, targeting a gentle ~8s damping settle.
  float I = (hw_.busInertiaTensor[0][0] + hw_.busInertiaTensor[1][1] + hw_.busInertiaTensor[2][2]) / 3.0f;
  float detumbleSettlingTime = 8.0f;
  float omega_n = 4.0f / detumbleSettlingTime;
  detumbleKd = 2.0f * I * omega_n;

  // B-dot gain: m_cmd = -bdotGain * dB/dt. There's no closed-form tuning
  // here (would need an orbital rate this class deliberately doesn't
  // know), so instead this targets near-max dipole moment at a
  // representative *ongoing* tumble, not the initial worst case.
  //
  // |dB/dt| = |omega x B_body|, which peaks at tumbleRate*fieldMagnitude
  // only when omega is exactly perpendicular to the field -- for a
  // generic relative orientation the expected value of that projection is
  // E[sin(theta)] = pi/4 for a uniformly random angle, not 1. Confirmed via
  // a headless sweep: using the peak (worst-case) value directly as the
  // tuning target left rods sitting at ~15-30% saturation for most of a
  // detumble, under-driving the actuator almost the entire time.
  //
  // It also targets a representative *ongoing* rate (0.3 rad/s) rather
  // than the initial deployment peak (~1 rad/s) -- the fast initial phase
  // self-corrects quickly regardless of exact saturation; the actuator's
  // utilization matters far more during the slower, longer "final
  // approach" that follows.
  {
    float maxMoment = hw_.torquers[0].maxMomentAm2;
    float representativeTumbleRateRadS = 0.3f;
    float representativeFieldT = 30e-6f;
    float expectedProjectionFactor = glm::quarter_pi<float>(); // E[sin(theta)], random relative orientation
    float representativeDbDt = representativeTumbleRateRadS * representativeFieldT * expectedProjectionFactor;
    bdotGain = maxMoment / representativeDbDt;
  }

  // Desaturation gain: m = (k/|B|^2) * (H_wheel x B), so |m| <=
  // k*|H_wheel|/|B|. Targets near-max dipole moment against a single wheel
  // sitting at its own max momentum in a representative LEO field -- the
  // point at which desaturation actually matters.
  //
  // The 2x is an empirically-chosen margin, not a physical constant:
  // updateDesaturation()'s headroom scaling already protects against the
  // instability a fixed gain alone caused (see its comment), but a
  // headless gain sweep at the worst-headroom starting point (90% wheel
  // saturation) showed the stability margin above ~3x becomes noisy --
  // some runs stable, some not, depending on the random sensor noise draw.
  // 2x stayed clearly stable across every seed tested while still
  // desaturating meaningfully faster than 1x.
  {
    float maxMoment = hw_.torquers[0].maxMomentAm2;
    float representativeWheelMomentum = hw_.wheels[0].wheelInertia * hw_.wheels[0].maxSpeedRadS;
    float representativeFieldT = 30e-6f;
    float stableSpeedMargin = 2.0f;
    desatGain = (representativeWheelMomentum > 1e-12f)
                    ? (stableSpeedMargin * maxMoment * representativeFieldT / representativeWheelMomentum)
                    : 0.0f;
  }
}

void ADCS::resetController()
{
  pid.reset();
  lqr.reset();
  cascaded.reset();
}

void ADCS::retuneForMode()
{
  ModeTuning t = tuningForMode(effectiveMode);
  pid.autoTune(hw_.busInertiaTensor, t.settlingTime, t.dampingRatio);
  lqr.autoTune(hw_.busInertiaTensor, t.settlingTime, t.dampingRatio, t.omega_max);
  cascaded.autoTune(hw_.busInertiaTensor, t.settlingTime, t.dampingRatio, t.omega_max);
  resetController(); // discard integral windup/state accumulated under the old tuning
}

FSWOutputs ADCS::step(const FSWInputs &in, float dt)
{
  lastGyroBody = in.imu.gyro;
  lastAccelBody = in.imu.accel;

  // EKF predict: propagates estimatedAttitude via bias-corrected strapdown
  // integration and grows the covariance -- see propagateEstimator().
  propagateEstimator(in.imu.gyro, dt);

  // Magnetometer: feeds both the B-dot law and (below) the TRIAD fallback
  // correction. Finite-differenced the same way a real accelerometer-from-
  // velocity derivation would be, across consecutive valid samples.
  if (in.mag.valid)
  {
    magFieldBody = in.mag.fieldBody;
    if (hasPrevMagField && dt > 1e-6f)
      magFieldRateBody = (magFieldBody - prevMagFieldBody) / dt;
    prevMagFieldBody = magFieldBody;
    hasPrevMagField = true;
  }

  // EKF correct: prefer the star tracker; fall back to the coarser TRIAD
  // solve when it's unavailable (sun-blinded, slewing too fast). If
  // neither is available this cycle, the estimate just keeps coasting on
  // the propagation above and covariance keeps growing -- exactly what a
  // real system does through a dropout.
  starTrackerValid = in.star.valid;
  triadFallbackUsed = false;

  if (in.star.valid)
  {
    // Star tracker noise spec would normally be a configured constant too
    // (see sunSensorNoiseRad); using the same ~10 arcsec figure the
    // simulated StarTracker defaults to, since this project doesn't yet
    // have a reason to make it independently configurable.
    constexpr float starTrackerNoiseRad = 10.0f / 3600.0f * 3.14159265f / 180.0f;
    correctEstimator(in.star.attitudeWorld, starTrackerNoiseRad * starTrackerNoiseRad);
  }
  else
  {
    glm::quat triadMeas;
    float triadR;
    if (computeTriadFallback(in, triadMeas, triadR))
    {
      correctEstimator(triadMeas, triadR);
      triadFallbackUsed = true;
    }
  }

  float attTraceRad2 = covAA[0][0] + covAA[1][1] + covAA[2][2];
  attitudeUncertaintyDeg = glm::degrees(std::sqrt(std::max(attTraceRad2, 0.0f) / 3.0f));

  // Rate feedback to the attitude controller: bias-corrected, same as
  // propagation above. A rate loop fed the raw (still-biased) gyro settles
  // at a true rate equal to minus that bias -- using the EKF's own bias
  // estimate here, not just for attitude propagation, is what actually
  // fixes that instead of only bounding its effect.
  glm::vec3 rate = in.imu.gyro - gyroBiasEstimate;

  // Autonomous mode manager / FDIR: decides what guidance/control actually
  // execute this cycle, using exactly the FSW-derived signals available at
  // this point (wheel health telemetry, the EKF's own confidence, and the
  // bias-corrected rate above) -- never anything this class itself
  // couldn't otherwise know. Runs even under manualOverride (a fault is
  // still worth detecting/logging while a human has the stick), it just
  // doesn't get to act until manual control is released -- see below.
  FdirInputs fdirIn;
  fdirIn.wheelTelemetry = in.wheelTelemetry;
  fdirIn.attitudeUncertaintyDeg = attitudeUncertaintyDeg;
  fdirIn.rateBody = rate;
  fdirIn.commandedMode = mode;
  effectiveMode = fdir.evaluate(fdirIn, dt);

  if (effectiveMode != lastTunedMode)
  {
    retuneForMode();
    lastTunedMode = effectiveMode;
  }

  if (manualOverride)
  {
    // Bypass guidance/control/allocation entirely -- a UI panel commanding
    // hardware directly, same clamping semantics the normal path would
    // apply, just skipping the autonomous loop.
    wheelCommands = manualWheelTorqueNm;
    magnetorquerCommands = manualMagnetorquerMomentAm2;
    return buildOutputs();
  }

  computeGuidance(in.spacecraftPositionWorld, dt);
  computeControl(estimatedAttitude, rate, dt);
  updateDesaturation(in);
  allocateActuators();
  return buildOutputs();
}

void ADCS::computeGuidance(const glm::vec3 &spacecraftPositionWorld, float dt)
{
  if (effectiveMode == PointingMode::DETUMBLE)
    return; // no attitude target; computeControl uses a rate-only law instead

  glm::vec3 pointDir;
  switch (effectiveMode)
  {
  case PointingMode::NADIR:
    // "Straight down" -- this sim has no orbital mechanics, so there's no
    // real nadir vector (Earth-center direction) to compute; a fixed world
    // direction is the honest equivalent here.
    pointDir = glm::vec3(0, 0, -1);
    break;
  case PointingMode::SUN_POINTING:
    pointDir = glm::normalize(sunPosition - spacecraftPositionWorld);
    break;
  case PointingMode::REFLECT:
  {
    // Aim the mirror's normal (body +Z) so it bounces sunlight onto
    // `target`. For a flat mirror, the normal that reflects a ray from A
    // to B is the bisector of the two directions away from the mirror --
    // normalize(dirToSun + dirToTarget) -- the same law drawSunReflection()
    // in the harness uses to draw the incident/reflected rays, just solved
    // for the normal instead of applied to it.
    glm::vec3 dirToSun = glm::normalize(sunPosition - spacecraftPositionWorld);
    glm::vec3 dirToTarget = glm::normalize(target - spacecraftPositionWorld);
    pointDir = glm::normalize(dirToSun + dirToTarget);
    break;
  }
  case PointingMode::TARGET:
  case PointingMode::SLEW:
  case PointingMode::FINE_POINTING:
  default:
    pointDir = glm::normalize(target - spacecraftPositionWorld);
    break;
  }

  glm::vec3 bodyUp{0, 0, 1}; // Local +Z axis

  // Rotation from +Z to the pointing direction
  float dot = glm::dot(bodyUp, pointDir);

  if (dot > 0.9999f)
  {
    targetAttitude = glm::quat{1, 0, 0, 0}; // Already aligned
  }
  else if (dot < -0.9999f)
  {
    // Opposite direction - rotate 180 degrees around any perpendicular axis
    targetAttitude = glm::angleAxis(glm::pi<float>(), glm::vec3{1, 0, 0});
  }
  else
  {
    glm::vec3 axis = glm::normalize(glm::cross(bodyUp, pointDir));
    float angle = acos(dot);
    targetAttitude = glm::angleAxis(angle, axis);
  }

  glm::quat estErrQ = glm::inverse(estimatedAttitude) * targetAttitude;
  if (estErrQ.w < 0.0f)
    estErrQ = -estErrQ;
  estimatedPointingErrorDeg = glm::degrees(2.0f * std::acos(glm::clamp(estErrQ.w, -1.0f, 1.0f)));
}

void ADCS::computeControl(glm::quat attitude, glm::vec3 rate, float dt)
{
  if (effectiveMode == PointingMode::DETUMBLE)
  {
    if (detumbleActuator == DetumbleActuator::MAGNETORQUERS_BDOT)
    {
      // Wheels get nothing this cycle -- allocateActuators() still runs
      // for them (torqueCommand stays whatever it last was otherwise), so
      // explicitly zero it rather than leave a stale command in place.
      torqueCommand = glm::vec3(0.0f);
      bdotDipoleCommandBody = computeBdotDipoleCommand();
    }
    else
    {
      torqueCommand = computeDetumbleTorque(rate);
      bdotDipoleCommandBody = glm::vec3(0.0f);
    }
    return;
  }
  bdotDipoleCommandBody = glm::vec3(0.0f); // magnetorquers idle outside DETUMBLE+B-dot

  switch (controllerType)
  {
  case ControllerType::LQR:
    torqueCommand = lqr.computeControlTorque(targetAttitude, attitude, rate, dt);
    break;
  case ControllerType::CASCADED:
    torqueCommand = cascaded.computeControlTorque(targetAttitude, attitude, rate, dt);
    break;
  case ControllerType::PID:
  default:
    torqueCommand = pid.computeControlTorque(targetAttitude, attitude, rate, dt);
    break;
  }
}

glm::vec3 ADCS::computeDetumbleTorque(const glm::vec3 &rate) const
{
  // Pure rate damping: right after deployment there's no attitude worth
  // chasing yet, only a rotation to kill.
  //
  // Sign note: the physically-damping torque is -Kd*rate, but
  // allocateActuators()/the wheel's own reaction dynamics apply the
  // *negative* of whatever torqueCommand they're given (Newton's-third-law
  // reaction convention). This needs the same compensating negation, so
  // the value returned here is +Kd*rate.
  return detumbleKd * rate;
}

glm::vec3 ADCS::computeBdotDipoleCommand() const
{
  // Classic B-dot law: m = -k * dB/dt. Driving the commanded dipole
  // opposite the field's own rate of change damps body rotation without
  // ever needing an attitude estimate (Wisniewski's B-dot detumbling, the
  // standard magnetics-only technique real cubesats use right after
  // deployment).
  return -bdotGain * magFieldRateBody;
}

void ADCS::updateDesaturation(const FSWInputs &in)
{
  desatDipoleCommandBody = glm::vec3(0.0f);

  // DETUMBLE already owns the magnetorquers for B-dot -- don't fight it
  // over the same hardware.
  if (effectiveMode == PointingMode::DETUMBLE)
  {
    desatActive = false;
    return;
  }

  glm::vec3 wheelMomentumBody(0.0f);
  float maxWheelSat = 0.0f;
  for (int i = 0; i < NUM_WHEELS; i++)
  {
    const WheelConfig &wc = hw_.wheels[i];
    float speed = in.wheelTelemetry[i].speedRadS;
    wheelMomentumBody += wc.wheelInertia * speed * wc.spinAxisBody;
    if (wc.maxSpeedRadS > 1e-9f)
      maxWheelSat = std::max(maxWheelSat, std::abs(speed / wc.maxSpeedRadS));
  }

  if (desatAutoTriggerEnabled && !desatActive && maxWheelSat >= desatTriggerSaturation)
    desatActive = true;

  if (!desatActive)
    return;

  if (maxWheelSat <= desatStopSaturation)
  {
    desatActive = false; // reached the (lower, hysteresis) stop level -- done
    return;
  }

  // Cross-product law (Stickler & Alfriend): commands a dipole moment
  // that, once the attitude controller reacts to the resulting external
  // torque, ends up reducing the wheels' stored momentum rather than the
  // spacecraft's attitude. Computed in body frame from the magnetometer
  // reading directly (same as the B-dot law above) -- what a real onboard
  // implementation would have to work with.
  //
  // Sign: textbook presentations of this law usually write
  // m = -(k/|B|^2)(h x B), assuming the actuator applies m x B directly
  // to the body. This sim's wheel reaction dynamics do not -- the
  // commanded wheel torque and the *actual* body torque are related by a
  // Newton's-third-law negation (see computeDetumbleTorque's sign note),
  // and working through that negation here flips which sign actually
  // drains h: dh_wheel/dt ends up equal to +(m x B), not -(m x B), so
  // draining requires m = +(k/|B|^2)(h x B). Confirmed empirically -- the
  // textbook-sign version pumped wheels to 100% saturation and
  // destabilized pointing (up to 170 deg error) in a headless test; this
  // sign drains the wheels while holding pointing steady.
  glm::vec3 B = magFieldBody;
  float bMagSq = glm::dot(B, B);
  if (bMagSq < 1e-18f)
    return; // no usable field reading yet

  // Headroom scaling: the resulting external torque has to be absorbed by
  // the *same* wheels being desaturated, so pushing the full-strength law
  // on a wheel that's already near maxSpeed leaves it no spare torque to
  // react with -- confirmed empirically: at fixed gain, starting a pass at
  // 90% saturation destabilized pointing (up to 170 deg error) and drove
  // wheels to 100%, while the identical gain starting from 85% stayed
  // well-behaved (under 3 deg). Scaling by remaining headroom lets desat
  // push hard when there's room to react and automatically back off as
  // that room disappears.
  float headroom = glm::clamp(1.0f - maxWheelSat, 0.05f, 1.0f);
  desatDipoleCommandBody = headroom * (desatGain / bMagSq) * glm::cross(wheelMomentumBody, B);
}

template <int N>
std::array<float, N> ADCS::allocateViaPseudoinverse(
    const glm::vec3 &command, const std::array<glm::vec3, N> &axesBody)
{
  std::array<float, N> out{};

  // Minimum-norm allocation via the Moore-Penrose pseudoinverse
  // A+ = A^T (A A^T)^-1, where A's columns are each actuator's axis (body
  // frame). A A^T is only 3x3 regardless of actuator count, so this
  // handles any N (3 orthogonal, a 4-rod pyramid, ...) as long as the axes
  // span all 3 dimensions.
  glm::mat3 AAt(0.0f);
  for (int i = 0; i < N; i++)
  {
    const glm::vec3 &a = axesBody[i];
    AAt += glm::mat3(a.x * a, a.y * a, a.z * a); // outer product a * a^T
  }

  glm::vec3 y = glm::inverse(AAt) * command;

  for (int i = 0; i < N; i++)
    out[i] = glm::dot(axesBody[i], y);
  return out;
}

void ADCS::allocateActuators()
{
  std::array<glm::vec3, NUM_WHEELS> wheelAxes;
  for (int i = 0; i < NUM_WHEELS; i++)
    wheelAxes[i] = hw_.wheels[i].spinAxisBody;
  wheelCommands = allocateViaPseudoinverse<NUM_WHEELS>(torqueCommand, wheelAxes);

  std::array<glm::vec3, NUM_TORQUERS> torquerAxes;
  for (int i = 0; i < NUM_TORQUERS; i++)
    torquerAxes[i] = hw_.torquers[i].axisBody;
  // B-dot and desaturation both command the magnetorquers, but never at
  // the same time in practice -- B-dot only during DETUMBLE, desat only
  // outside it -- so summing them is exactly "whichever one is currently
  // nonzero."
  magnetorquerCommands = allocateViaPseudoinverse<NUM_TORQUERS>(bdotDipoleCommandBody + desatDipoleCommandBody, torquerAxes);
}

FSWOutputs ADCS::buildOutputs() const
{
  FSWOutputs out;
  for (int i = 0; i < NUM_WHEELS; i++)
  {
    float t = glm::clamp(wheelCommands[i], -hw_.wheels[i].maxTorqueNm, hw_.wheels[i].maxTorqueNm);
    out.wheelCommands[i] = {t};
  }
  for (int i = 0; i < NUM_TORQUERS; i++)
  {
    float m = glm::clamp(magnetorquerCommands[i], -hw_.torquers[i].maxMomentAm2, hw_.torquers[i].maxMomentAm2);
    out.torquerCommands[i] = {m};
  }
  return out;
}

glm::quat ADCS::computeTriadAttitude(const glm::vec3 &primaryBody, const glm::vec3 &primaryRef,
                                     const glm::vec3 &secondaryBody, const glm::vec3 &secondaryRef)
{
  glm::vec3 tb1 = glm::normalize(primaryBody);
  glm::vec3 tb2 = glm::normalize(glm::cross(primaryBody, secondaryBody));
  glm::vec3 tb3 = glm::cross(tb1, tb2);

  glm::vec3 tr1 = glm::normalize(primaryRef);
  glm::vec3 tr2 = glm::normalize(glm::cross(primaryRef, secondaryRef));
  glm::vec3 tr3 = glm::cross(tr1, tr2);

  // Columns are each triad's basis vectors. Mref * transpose(Mbody) maps
  // the body-frame triad onto the reference-frame triad exactly (both are
  // orthonormal by construction), i.e. it's the body-to-world rotation.
  glm::mat3 Mbody(tb1, tb2, tb3);
  glm::mat3 Mref(tr1, tr2, tr3);
  glm::mat3 R = Mref * glm::transpose(Mbody);

  return glm::normalize(glm::quat_cast(R));
}

void ADCS::propagateEstimator(const glm::vec3 &gyroMeasured, float dt)
{
  glm::vec3 omega = gyroMeasured - gyroBiasEstimate;

  // Attitude: strapdown quaternion kinematics using the bias-corrected
  // rate.
  glm::quat omegaQuat(0.0f, omega.x, omega.y, omega.z);
  estimatedAttitude = estimatedAttitude + 0.5f * estimatedAttitude * omegaQuat * dt;
  estimatedAttitude = glm::normalize(estimatedAttitude);

  // Covariance: first-order (Euler) discretization of the standard
  // attitude+bias error-state dynamics
  //   d(deltaTheta)/dt = -omega x deltaTheta - deltaBias
  //   d(deltaBias)/dt  = 0   (pure random walk, driven only by process noise)
  // i.e. Phi = I + F*dt with F = [[-[omega x], -I], [0, 0]], applied in
  // block form. Derived by hand-expanding Phi*P*Phi^T + Q*dt in blocks and
  // cross-checked for self-consistency (newPab must equal
  // transpose(newPba) since P stays symmetric).
  glm::mat3 I3(1.0f);
  glm::mat3 phiAA = I3 - skewSymmetric(omega) * dt;

  glm::mat3 newAA = phiAA * covAA * glm::transpose(phiAA)
                   - dt * glm::transpose(covAB) * glm::transpose(phiAA)
                   - dt * phiAA * covAB
                   + dt * dt * covBB
                   + gyroNoisePsd * dt * I3;
  glm::mat3 newAB = phiAA * covAB - dt * covBB;
  glm::mat3 newBB = covBB + gyroBiasWalkPsd * dt * I3;

  covAA = newAA;
  covAB = newAB;
  covBB = newBB;
}

void ADCS::correctEstimator(const glm::quat &qMeas, float R)
{
  glm::quat qErr = glm::inverse(estimatedAttitude) * qMeas;
  if (qErr.w < 0.0f)
    qErr = -qErr;
  glm::vec3 dz(2.0f * qErr.x, 2.0f * qErr.y, 2.0f * qErr.z); // innovation: small-angle attitude error

  glm::mat3 I3(1.0f);
  glm::mat3 S = covAA + glm::mat3(R); // H = [I, 0], so H*P*H^T = covAA; isotropic measurement noise R*I
  glm::mat3 Sinv = glm::inverse(S);

  glm::mat3 Ka = covAA * Sinv;                 // top block of Kalman gain K = P*H^T*S^-1
  glm::mat3 Kb = glm::transpose(covAB) * Sinv; // bottom block (covBA = transpose(covAB))

  glm::vec3 dTheta = Ka * dz;
  glm::vec3 dBias = Kb * dz;

  // Multiplicative reset: fold the small-angle correction directly into
  // the quaternion via the same small-angle approximation used throughout
  // this class (and by the simulated StarTracker's own noise model).
  glm::quat dq = glm::normalize(glm::quat(1.0f, 0.5f * dTheta.x, 0.5f * dTheta.y, 0.5f * dTheta.z));
  estimatedAttitude = glm::normalize(estimatedAttitude * dq);
  gyroBiasEstimate += dBias;

  // Covariance: P_new = (I - K*H)*P, expanded in blocks (H = [I, 0]).
  glm::mat3 newAA = (I3 - Ka) * covAA;
  glm::mat3 newAB = (I3 - Ka) * covAB;
  glm::mat3 newBB = covBB - Kb * covAB;

  covAA = newAA;
  covAB = newAB;
  covBB = newBB;
}

bool ADCS::computeTriadFallback(const FSWInputs &in, glm::quat &outAttitude, float &outR) const
{
  if (!in.mag.valid || glm::length(in.mag.fieldBody) < 1e-12f || glm::length(ambientFieldWorld) < 1e-9f)
    return false;
  if (!in.sunSensor.valid)
    return false;

  glm::vec3 sunDirRef = sunPosition - in.spacecraftPositionWorld;
  if (glm::length(sunDirRef) < 1e-6f)
    return false; // no sun position configured -- can't form the second vector
  sunDirRef = glm::normalize(sunDirRef);

  // TRIAD is singular (and noisy near-singular) when the two references
  // are nearly parallel -- report no valid measurement rather than solve
  // against an ill-conditioned pair.
  glm::vec3 fieldDirRef = glm::normalize(ambientFieldWorld);
  if (glm::length(glm::cross(fieldDirRef, sunDirRef)) < 0.1f)
    return false;

  outAttitude = computeTriadAttitude(in.mag.fieldBody, ambientFieldWorld, in.sunSensor.sunDirBody, sunDirRef);
  // Dominated by the coarse sun sensor's noise (the magnetometer is
  // comparatively accurate here) -- an approximation, not a rigorous
  // propagation of both sensors' noise through the TRIAD solve.
  outR = sunSensorNoiseRad * sunSensorNoiseRad;
  return true;
}
