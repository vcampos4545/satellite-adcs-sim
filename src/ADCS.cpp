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
  default:
    return {5.0f, 1.0f, 0.5f};
  }
}
} // namespace

ADCS::ADCS(RigidBody *body_, const std::vector<ReactionWheel *> &wheels_, IMU *imu_,
          const std::vector<Magnetorquer *> &magnetorquers_, Magnetometer *magnetometer_,
          StarTracker *starTracker_)
{
  body = body_;
  wheels = wheels_;
  imu = imu_;
  magnetorquers = magnetorquers_;
  magnetometer = magnetometer_;
  starTracker = starTracker_;

  // Bootstrap: even with a star tracker, the very first attitude has to
  // come from somewhere before any sample has been taken. Starting from
  // truth is the one place ground truth is used -- every update after
  // this comes from the gyro (propagateEstimator) and the star tracker/
  // TRIAD fallback (correctEstimator), never body->orientation again.
  estimatedAttitude = body->orientation;
  gyroBiasEstimate = glm::vec3(0.0f);

  // EKF covariance priors. Attitude starts with a modest nonzero
  // uncertainty even though it was just seeded from truth above -- a real
  // system wouldn't have that luxury, and starting at literally zero
  // would make the very first Kalman gain computation divide by
  // (numerically) nothing. Bias uncertainty is seeded from the IMU's own
  // turn-on bias range, reusing a number the sensor model already commits
  // to rather than picking an arbitrary one here.
  float initialAttitudeUncertaintyRad = glm::radians(1.0f);
  covAA = glm::mat3(initialAttitudeUncertaintyRad * initialAttitudeUncertaintyRad);
  covAB = glm::mat3(0.0f);
  if (imu)
  {
    float biasRangeRad = (imu->gyroBiasRange.x + imu->gyroBiasRange.y + imu->gyroBiasRange.z) / 3.0f;
    covBB = glm::mat3(biasRangeRad * biasRangeRad);

    // Process noise PSDs, approximated directly from the IMU's own
    // per-sample noise/drift figures (not a rigorous PSD derivation --
    // consistent with this sim's overall fidelity level elsewhere, e.g.
    // the B-dot/desat gains are also order-of-magnitude heuristics, not
    // closed-form results).
    float gyroNoiseAvg = (imu->gyroNoiseStd.x + imu->gyroNoiseStd.y + imu->gyroNoiseStd.z) / 3.0f;
    float gyroBiasDriftAvg = (imu->gyroBiasDriftStd.x + imu->gyroBiasDriftStd.y + imu->gyroBiasDriftStd.z) / 3.0f;
    gyroNoisePsd = gyroNoiseAvg * gyroNoiseAvg;
    gyroBiasWalkPsd = gyroBiasDriftAvg * gyroBiasDriftAvg;
  }
  else
  {
    covBB = glm::mat3(1.0f);
  }

  lastTunedMode = mode;
  retuneForMode();

  // Detumble's rate-damping gain isn't part of the quaternion-error
  // controllers' tuning; derive it from inertia the same way autoTune
  // derives Kd, targeting a gentle ~8s damping settle.
  float I = (body->inertiaTensor[0][0] + body->inertiaTensor[1][1] + body->inertiaTensor[2][2]) / 3.0f;
  float detumbleSettlingTime = 8.0f;
  float omega_n = 4.0f / detumbleSettlingTime;
  detumbleKd = 2.0f * I * omega_n;

  // B-dot gain: m_cmd = -bdotGain * dB/dt. There's no closed-form tuning
  // here the way the attitude controllers have (that needs an orbital rate,
  // which this class deliberately doesn't know -- see ADCS.h), so instead
  // this targets near-max dipole moment at a representative *ongoing*
  // tumble, not the initial worst case.
  //
  // |dB/dt| = |omega x B_body|, which peaks at tumbleRate*fieldMagnitude
  // only when omega is exactly perpendicular to the field -- for a
  // generic relative orientation the expected value of that projection is
  // E[sin(theta)] = pi/4 for a uniformly random angle, not 1. Originally
  // this used the peak (worst-case) value directly as the tuning target,
  // which (confirmed via a headless sweep: default gain left rods sitting
  // at ~15-30% saturation for most of a detumble, only briefly touching
  // higher values) under-drove the actuator almost the entire time,
  // needlessly slowing detumble.
  //
  // It also targeted the *initial* deployment tumble rate (~1 rad/s),
  // but that fast initial phase self-corrects quickly regardless of
  // exactly how saturated the rods are -- the actuator's utilization
  // matters far more during the slower, longer "final approach" that
  // follows. Tuning around a representative ongoing rate (0.3 rad/s)
  // instead of the initial peak keeps the gain well-matched to where the
  // bulk of detumble time is actually spent.
  if (!magnetorquers.empty())
  {
    float maxMoment = magnetorquers[0]->maxDipoleMoment;
    float representativeTumbleRateRadS = 0.3f;
    float representativeFieldT = 30e-6f;
    float expectedProjectionFactor = glm::quarter_pi<float>(); // E[sin(theta)], random relative orientation
    float representativeDbDt = representativeTumbleRateRadS * representativeFieldT * expectedProjectionFactor;
    bdotGain = maxMoment / representativeDbDt;
  }

  // Desaturation gain: m = (k/|B|^2) * (H_wheel x B), so |m| <=
  // k*|H_wheel|/|B|. No closed-form tuning here either (would need the
  // disturbance torque profile this class doesn't model), so this
  // targets near-max dipole moment against a single wheel sitting at its
  // own max momentum in a representative LEO field -- the point at which
  // desaturation actually matters.
  //
  // The 2x is an empirically-chosen margin, not a physical constant:
  // updateDesaturation()'s headroom scaling already protects against the
  // instability a fixed gain alone caused (see its comment), but a
  // headless gain sweep at the worst-headroom starting point (90% wheel
  // saturation) showed the stability margin above ~3x becomes noisy --
  // some runs stable, some not, depending on the random sensor noise
  // draw. 2x stayed clearly stable across every seed tested while still
  // desaturating meaningfully faster than 1x.
  if (!magnetorquers.empty() && !wheels.empty())
  {
    float maxMoment = magnetorquers[0]->maxDipoleMoment;
    float representativeWheelMomentum = wheels[0]->wheelInertia * wheels[0]->maxSpeed;
    float representativeFieldT = 30e-6f;
    float stableSpeedMargin = 2.0f;
    desatGain = (representativeWheelMomentum > 1e-12f)
                    ? (stableSpeedMargin * maxMoment * representativeFieldT / representativeWheelMomentum)
                    : 0.0f;
  }
}

ADCS::~ADCS() = default;

void ADCS::resetController()
{
  pid.reset();
  lqr.reset();
  cascaded.reset();
}

void ADCS::retuneForMode()
{
  ModeTuning t = tuningForMode(mode);
  pid.autoTune(body->inertiaTensor, t.settlingTime, t.dampingRatio);
  lqr.autoTune(body->inertiaTensor, t.settlingTime, t.dampingRatio, t.omega_max);
  cascaded.autoTune(body->inertiaTensor, t.settlingTime, t.dampingRatio, t.omega_max);
  resetController(); // discard integral windup/state accumulated under the old tuning
}

void ADCS::run(float dt)
{
  if (mode != lastTunedMode)
  {
    retuneForMode();
    lastTunedMode = mode;
  }

  // Sensors: the IMU is the only source of rate information here. Ground
  // truth (body->orientation / body->angularVelocity) is never read past
  // the initial bootstrap above.
  IMU::Reading imuReading = imu->sample(*body, gravity, dt);
  lastGyroBody = imuReading.gyro;
  lastAccelBody = imuReading.accel;

  // EKF predict: propagates estimatedAttitude via bias-corrected strapdown
  // integration and grows the covariance -- see propagateEstimator().
  propagateEstimator(imuReading.gyro, dt);

  // Magnetometer: feeds both the B-dot law and (below) the TRIAD fallback
  // correction. Finite-differenced the same way IMU's accelerometer
  // derives acceleration from consecutive velocity samples.
  if (magnetometer)
  {
    Magnetometer::Reading magReading = magnetometer->sample(*body, ambientFieldWorld, dt);
    magFieldBody = magReading.field;
    if (hasPrevMagField && dt > 1e-6f)
      magFieldRateBody = (magFieldBody - prevMagFieldBody) / dt;
    prevMagFieldBody = magFieldBody;
    hasPrevMagField = true;
  }

  // EKF correct: prefer the star tracker; fall back to the coarser TRIAD
  // solve when it's unavailable (sun-blinded, slewing too fast -- see
  // StarTracker.h). If neither is available this cycle, the estimate just
  // keeps coasting on the propagation above and covariance keeps growing
  // -- exactly what a real system does through a dropout.
  StarTracker::Reading starReading;
  if (starTracker)
    starReading = starTracker->sample(*body, sunPosition - body->position);
  starTrackerValid = starReading.valid;
  triadFallbackUsed = false;

  if (starReading.valid)
  {
    correctEstimator(starReading.attitude, starTracker->noiseStdRad * starTracker->noiseStdRad);
  }
  else
  {
    glm::quat triadMeas;
    float triadR;
    if (computeTriadFallback(triadMeas, triadR))
    {
      correctEstimator(triadMeas, triadR);
      triadFallbackUsed = true;
    }
  }

  float attTraceRad2 = covAA[0][0] + covAA[1][1] + covAA[2][2];
  attitudeUncertaintyDeg = glm::degrees(std::sqrt(std::max(attTraceRad2, 0.0f) / 3.0f));

  // Rate feedback to the attitude controller: bias-corrected, same as
  // propagation above. A rate loop fed the raw (still-biased) gyro
  // settles at a true rate equal to minus that bias (see
  // gyroBiasEstimate's comment in ADCS.h) -- using the EKF's own bias
  // estimate here, not just for the attitude propagation, is what
  // actually fixes that instead of only bounding its effect.
  glm::vec3 rate = imuReading.gyro - gyroBiasEstimate;

  if (manualOverride)
  {
    // Bypass guidance/control/allocation entirely -- a UI panel commanding
    // hardware directly, same actuators, same commandTorque()/
    // commandDipoleMoment() clamping, just skipping the autonomous loop.
    wheelCommands = manualWheelTorqueNm;
    wheelCommands.resize(wheels.size(), 0.0f);
    magnetorquerCommands = manualMagnetorquerMomentAm2;
    magnetorquerCommands.resize(magnetorquers.size(), 0.0f);
    sendCommands();
    return;
  }

  computeGuidance(dt);
  computeControl(estimatedAttitude, rate, dt);
  updateDesaturation();
  allocateActuators();
  sendCommands();
}

void ADCS::computeGuidance(float dt)
{
  if (mode == PointingMode::DETUMBLE)
    return; // no attitude target; computeControl uses a rate-only law instead

  glm::vec3 pointDir;
  switch (mode)
  {
  case PointingMode::NADIR:
    // "Straight down" -- this sim has no orbital mechanics, so there's no
    // real nadir vector (Earth-center direction) to compute; a fixed world
    // direction is the honest equivalent here.
    pointDir = glm::vec3(0, 0, -1);
    break;
  case PointingMode::SUN_POINTING:
    pointDir = glm::normalize(sunPosition - body->position);
    break;
  case PointingMode::TARGET:
  case PointingMode::SLEW:
  case PointingMode::FINE_POINTING:
  default:
    pointDir = glm::normalize(target - body->position);
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
    // Opposite direction - rotate 180° around any perpendicular axis
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

  // Ground truth -- for telemetry/diagnostics only (see ADCS.h); never
  // read back into guidance or control.
  glm::quat trueErrQ = glm::inverse(body->orientation) * targetAttitude;
  if (trueErrQ.w < 0.0f)
    trueErrQ = -trueErrQ;
  truePointingErrorDeg = glm::degrees(2.0f * std::acos(glm::clamp(trueErrQ.w, -1.0f, 1.0f)));
}

void ADCS::computeControl(glm::quat attitude, glm::vec3 rate, float dt)
{
  if (mode == PointingMode::DETUMBLE)
  {
    if (detumbleActuator == DetumbleActuator::MAGNETORQUERS_BDOT && !magnetorquers.empty())
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
  // chasing yet, only a rotation to kill. A wheel-only detumble can soak
  // up momentum but never dump it (that needs magnetorquers, added
  // later), so it will eventually saturate on a body with real residual
  // momentum -- that's expected, not a bug in this control law.
  //
  // Sign note: the physically-damping torque is -Kd*rate, but
  // allocateActuators()/ReactionWheel::apply() apply the *negative* of
  // whatever torqueCommand they're given (Newton's-third-law reaction
  // convention -- confirmed by direct measurement, not just derivation).
  // PIDController::computeControlTorque already returns its torque
  // pre-negated to compensate for that; this needs the same compensating
  // negation, so the value returned here is +Kd*rate.
  return detumbleKd * rate;
}

glm::vec3 ADCS::computeBdotDipoleCommand() const
{
  // Classic B-dot law: m = -k * dB/dt. Driving the commanded dipole
  // opposite the field's own rate of change damps body rotation without
  // ever needing an attitude estimate -- as the body tumbles, the
  // body-frame field appears to rotate too, and countering that rotation
  // countering the tumble (Wisniewski's B-dot detumbling, the standard
  // magnetics-only technique real cubesats use right after deployment).
  return -bdotGain * magFieldRateBody;
}

void ADCS::updateDesaturation()
{
  desatDipoleCommandBody = glm::vec3(0.0f);

  // DETUMBLE already owns the magnetorquers for B-dot -- don't fight it
  // over the same hardware.
  if (mode == PointingMode::DETUMBLE || magnetorquers.empty() || wheels.empty())
  {
    desatActive = false;
    return;
  }

  glm::vec3 wheelMomentumBody(0.0f);
  float maxWheelSat = 0.0f;
  for (auto *w : wheels)
  {
    wheelMomentumBody += w->wheelInertia * w->currentSpeed * w->spinAxisBody;
    maxWheelSat = std::max(maxWheelSat, std::abs(w->getSaturationRatio()));
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
  // reading directly (same as the B-dot law above), not the "true"
  // ambientFieldWorld -- this is what a real onboard implementation would
  // have to work with.
  //
  // Sign: textbook presentations of this law usually write
  // m = -(k/|B|^2)(h x B), assuming the actuator applies m x B directly
  // to the body. This sim's ReactionWheel does not -- the attitude
  // controller's commanded wheel torque and the *actual* body torque are
  // related by a Newton's-third-law negation (see computeDetumbleTorque's
  // sign note), and working through that negation here flips which sign
  // actually drains h: dh_wheel/dt ends up equal to +(m x B), not -(m x B),
  // so draining requires m = +(k/|B|^2)(h x B). Confirmed empirically --
  // the textbook-sign version pumped wheels to 100% saturation and
  // destabilized pointing (up to 170 deg error) in a headless test; this
  // sign drains the wheels while holding pointing steady.
  glm::vec3 B = magFieldBody;
  float bMagSq = glm::dot(B, B);
  if (bMagSq < 1e-18f)
    return; // no usable field reading yet

  // Headroom scaling: the resulting external torque has to be absorbed by
  // the *same* wheels being desaturated (the attitude controller reacts to
  // it by adjusting wheel torque), so pushing the full-strength law on a
  // wheel that's already near maxSpeed leaves it no spare torque to react
  // with -- confirmed empirically: at fixed gain, starting a pass at 90%
  // saturation destabilized pointing (up to 170 deg error) and drove
  // wheels to 100%, while the identical gain starting from 85% stayed
  // well-behaved (under 3 deg). Scaling by remaining headroom lets desat
  // push hard when there's room to react and automatically back off as
  // that room disappears, rather than needing a single fixed gain that's
  // only safe for the worst (least headroom) case throughout.
  float headroom = glm::clamp(1.0f - maxWheelSat, 0.05f, 1.0f);
  desatDipoleCommandBody = headroom * (desatGain / bMagSq) * glm::cross(wheelMomentumBody, B);
}

std::vector<float> ADCS::allocateViaPseudoinverse(
    const glm::vec3 &command, const std::vector<glm::vec3> &axesBody)
{
  int N = (int)axesBody.size();
  std::vector<float> out(N, 0.0f);
  if (N == 0)
    return out;

  // Minimum-norm allocation via the Moore-Penrose pseudoinverse
  // A+ = A^T (A A^T)^-1, where A's columns are each actuator's axis (body
  // frame). A A^T is only 3x3 regardless of actuator count, so this
  // handles any N (3 orthogonal, a 4-rod pyramid, ...) as long as the
  // axes span all 3 dimensions. For N == 3 orthogonal axes A A^T == I, so
  // this reduces to exactly the direct-inverse case.
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
  std::vector<glm::vec3> wheelAxes;
  for (auto *w : wheels)
    wheelAxes.push_back(w->spinAxisBody);
  wheelCommands = allocateViaPseudoinverse(torqueCommand, wheelAxes);

  std::vector<glm::vec3> torquerAxes;
  for (auto *m : magnetorquers)
    torquerAxes.push_back(m->axisBody);
  // B-dot and desaturation both command the magnetorquers, but never at
  // the same time in practice -- B-dot only during DETUMBLE, desat only
  // outside it (see updateDesaturation()) -- so summing them is exactly
  // "whichever one is currently nonzero."
  magnetorquerCommands = allocateViaPseudoinverse(bdotDipoleCommandBody + desatDipoleCommandBody, torquerAxes);
}

void ADCS::sendCommands()
{
  for (int i = 0; i < (int)wheels.size(); i++)
    wheels[i]->commandTorque(wheelCommands[i]);
  for (int i = 0; i < (int)magnetorquers.size(); i++)
    magnetorquers[i]->commandDipoleMoment(magnetorquerCommands[i]);
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
  // orthonormal by construction), i.e. it's the body-to-world rotation --
  // the same convention RigidBody::orientation uses.
  glm::mat3 Mbody(tb1, tb2, tb3);
  glm::mat3 Mref(tr1, tr2, tr3);
  glm::mat3 R = Mref * glm::transpose(Mbody);

  return glm::normalize(glm::quat_cast(R));
}

namespace
{
// S such that S*x == cross(v, x) for any x -- see ADCS.cpp's EKF comment
// for the F-matrix this builds. Verified against glm::cross() in a
// headless test before use here; glm::mat3's 9-scalar constructor is
// column-major, which is easy to get backwards.
glm::mat3 skewSymmetric(const glm::vec3 &v)
{
  return glm::mat3(0.0f, v.z, -v.y,
                   -v.z, 0.0f, v.x,
                    v.y, -v.x, 0.0f);
}
} // namespace

void ADCS::propagateEstimator(const glm::vec3 &gyroMeasured, float dt)
{
  glm::vec3 omega = gyroMeasured - gyroBiasEstimate;

  // Attitude: same strapdown quaternion kinematics as before, just using
  // the bias-corrected rate instead of the raw (biased) gyro reading now
  // that there's a bias estimate to correct with.
  glm::quat omegaQuat(0.0f, omega.x, omega.y, omega.z);
  estimatedAttitude = estimatedAttitude + 0.5f * estimatedAttitude * omegaQuat * dt;
  estimatedAttitude = glm::normalize(estimatedAttitude);

  // Covariance: first-order (Euler) discretization of the standard
  // attitude+bias error-state dynamics
  //   d(deltaTheta)/dt = -omega x deltaTheta - deltaBias
  //   d(deltaBias)/dt  = 0   (pure random walk, driven only by process noise)
  // i.e. Phi = I + F*dt with F = [[-[omega x], -I], [0, 0]], applied in
  // block form (see covAA/covAB/covBB's comment in ADCS.h). Derived by
  // hand-expanding Phi*P*Phi^T + Q*dt in blocks and cross-checked for
  // self-consistency (newPab must equal transpose(newPba) since P stays
  // symmetric -- confirmed algebraically before trusting this in code).
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

  glm::mat3 Ka = covAA * Sinv;                        // top block of Kalman gain K = P*H^T*S^-1
  glm::mat3 Kb = glm::transpose(covAB) * Sinv;         // bottom block (covBA = transpose(covAB))

  glm::vec3 dTheta = Ka * dz;
  glm::vec3 dBias = Kb * dz;

  // Multiplicative reset: fold the small-angle correction directly into
  // the quaternion via the same small-angle approximation used
  // throughout this class (and by StarTracker's own noise model).
  glm::quat dq = glm::normalize(glm::quat(1.0f, 0.5f * dTheta.x, 0.5f * dTheta.y, 0.5f * dTheta.z));
  estimatedAttitude = glm::normalize(estimatedAttitude * dq);
  gyroBiasEstimate += dBias;

  // Covariance: P_new = (I - K*H)*P, expanded in blocks (H = [I, 0]).
  // Symmetry cross-checked the same way as propagateEstimator()'s update.
  glm::mat3 newAA = (I3 - Ka) * covAA;
  glm::mat3 newAB = (I3 - Ka) * covAB;
  glm::mat3 newBB = covBB - Kb * covAB;

  covAA = newAA;
  covAB = newAB;
  covBB = newBB;
}

bool ADCS::computeTriadFallback(glm::quat &outAttitude, float &outR) const
{
  if (!magnetometer || glm::length(magFieldBody) < 1e-12f || glm::length(ambientFieldWorld) < 1e-9f)
    return false;

  glm::vec3 sunDirRef = sunPosition - body->position;
  if (glm::length(sunDirRef) < 1e-6f)
    return false; // no sun position configured -- can't form the second vector
  sunDirRef = glm::normalize(sunDirRef);

  // Coarse sun sensor: not a dedicated sensor class (see ADCS.h), just the
  // true body-frame direction plus a small additive Gaussian perturbation
  // so this fallback measurement isn't an oracle.
  glm::vec3 sunDirBodyTrue = glm::inverse(body->orientation) * sunDirRef;
  std::normal_distribution<float> sunNoise(0.0f, sunSensorNoiseRad);
  glm::vec3 sunDirBody = glm::normalize(sunDirBodyTrue +
      glm::vec3(sunNoise(sunSensorRng), sunNoise(sunSensorRng), sunNoise(sunSensorRng)));

  // TRIAD is singular (and noisy near-singular) when the two references
  // are nearly parallel -- report no valid measurement rather than solve
  // against an ill-conditioned pair.
  glm::vec3 fieldDirRef = glm::normalize(ambientFieldWorld);
  if (glm::length(glm::cross(fieldDirRef, sunDirRef)) < 0.1f)
    return false;

  outAttitude = computeTriadAttitude(magFieldBody, ambientFieldWorld, sunDirBody, sunDirRef);
  // Dominated by the coarse sun sensor's noise (the magnetometer is
  // comparatively accurate here) -- an approximation, not a rigorous
  // propagation of both sensors' noise through the TRIAD solve, but
  // consistent with this class's other order-of-magnitude gain/noise
  // figures.
  outR = sunSensorNoiseRad * sunSensorNoiseRad;
  return true;
}
