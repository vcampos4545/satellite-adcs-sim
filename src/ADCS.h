#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <rigidbody/actuators/ReactionWheel.h>
#include <rigidbody/actuators/Magnetorquer.h>
#include <rigidbody/sensors/IMU.h>
#include <rigidbody/sensors/Magnetometer.h>
#include <rigidbody/sensors/StarTracker.h>
#include <rigidbody/RigidBody.h>
#include "Controllers.h"
#include <vector>
#include <random>

// Pointing modes. Nadir/Sun/Target/Slew/Fine-pointing all resolve to a
// target attitude and go through the normal PID/LQR/cascaded quaternion
// controller (with different tuning per mode -- see modeTuning() in
// ADCS.cpp); Detumble bypasses attitude control entirely and just damps
// rate, since a freshly-deployed, tumbling satellite has no attitude
// reference worth chasing yet.
enum class PointingMode
{
  NADIR,         // body +Z toward a fixed "down" direction (no orbit in this sim, so no real nadir vector)
  SUN_POINTING,  // body +Z toward the sun sphere
  DETUMBLE,      // damp angular rate toward zero; ignores attitude entirely
  TARGET,        // body +Z toward `target`, default tuning
  SLEW,          // body +Z toward `target`, tuned for a fast large-angle move
  FINE_POINTING, // body +Z toward `target`, tuned for a slow, precise settle
};

enum class ControllerType
{
  PID,
  LQR,
  CASCADED
};

// Which hardware DETUMBLE damps rate with. Reaction wheels are the default
// (fast, precise, works regardless of field strength); MAGNETORQUERS_BDOT
// switches to the classic B-dot law (m = -k * dB/dt, sensed by the
// magnetometer) instead -- the standard low-cost/low-mass way a real
// cubesat detumbles right after deployment, before wheels are even trusted.
// Only meaningful while mode == DETUMBLE; ignored otherwise.
enum class DetumbleActuator
{
  REACTION_WHEELS,
  MAGNETORQUERS_BDOT
};

class ADCS
{
public:
  PointingMode mode = PointingMode::TARGET;
  ControllerType controllerType = ControllerType::PID;
  DetumbleActuator detumbleActuator = DetumbleActuator::REACTION_WHEELS;

  glm::vec3 target = {0.5f, 0.5f, 0.5f};      // world position for TARGET/SLEW/FINE_POINTING
  glm::vec3 sunPosition = {0.0f, 0.0f, 0.0f}; // world position for SUN_POINTING

  glm::quat targetAttitude;

  // Pointing error (degrees) against targetAttitude, updated by
  // computeGuidance() -- held at its last value during DETUMBLE, which has
  // no attitude target. estimatedPointingErrorDeg is what the FSW itself
  // actually computes and could act on (against estimatedAttitude);
  // truePointingErrorDeg is ground truth (against body->orientation),
  // exposed purely for telemetry/diagnostics -- never read by guidance or
  // control, same boundary the rest of this class keeps around ground
  // truth elsewhere.
  float estimatedPointingErrorDeg = 0.0f;
  float truePointingErrorDeg = 0.0f;

  glm::vec3 torqueCommand;
  std::vector<float> wheelCommands;         // wheelCommands[i] -> torque for wheels[i] (Nm)
  std::vector<float> magnetorquerCommands;  // magnetorquerCommands[i] -> dipole moment for magnetorquers[i] (A*m^2)

  // B-dot gain (A*m^2 per T/s): m_cmd = -bdotGain * dB/dt. Tuned once at
  // construction from the magnetorquers' max moment (see ADCS.cpp); exposed
  // here so a UI panel can retune it live.
  float bdotGain = 0.0f;

  // Manual actuator override: when true, run() skips guidance/control/
  // allocation entirely and sends manualWheelTorqueNm/
  // manualMagnetorquerMomentAm2 straight to hardware -- for a UI panel that
  // wants direct per-actuator sliders instead of going through a pointing
  // mode. Sized independently of wheels/magnetorquers; sendCommands() reads
  // whichever indices exist, so a short vector just leaves the remaining
  // actuators uncommanded (zero).
  bool manualOverride = false;
  std::vector<float> manualWheelTorqueNm;
  std::vector<float> manualMagnetorquerMomentAm2;

  // Attitude estimate, propagated by strapdown-integrating the (bias-
  // corrected) gyro reading every run() and periodically corrected
  // against an absolute reference -- see propagateEstimator()/
  // correctEstimator() and the multiplicative-EKF comment below. This is
  // what guidance/control actually use; ground truth (body->orientation)
  // is never read here.
  glm::quat estimatedAttitude{1, 0, 0, 0};

  // Gyro bias estimate (rad/s, body frame) -- the EKF's second state
  // besides attitude. Subtracted from every raw gyro reading before it's
  // used for propagation *or* rate feedback to the attitude controller
  // (see run()): a rate loop that nulls the raw, still-biased measurement
  // settles at a true rate equal to minus that bias, a real steady-state
  // pointing error tuning alone can't fix (this was chased down and
  // worked around with a periodic TRIAD correction a few iterations ago;
  // actually estimating and removing the bias, rather than just bounding
  // its effect after the fact, is the more direct fix).
  glm::vec3 gyroBiasEstimate{0.0f};

  // 1-sigma attitude uncertainty (degrees), derived from the EKF
  // covariance's attitude block -- exposed so FSW/UI can judge how much
  // to trust the estimate right now, not just read a point value with no
  // sense of its own confidence. Grows during star-tracker dropouts
  // (sun-blinded, slewing too fast) and shrinks again once a correction
  // lands -- this is the mechanism a real system would use to know it's
  // approaching the edge of a pointing budget before actually violating
  // it, rather than finding out after the fact.
  float attitudeUncertaintyDeg = 0.0f;

  // Whether the star tracker had a valid reading this cycle (vs. blinded
  // by the sun or slewing too fast) and, if not, whether the TRIAD
  // fallback supplied a correction instead -- exposed for telemetry/UI so
  // it's visible when the estimate is coasting on propagation alone.
  bool starTrackerValid = false;
  bool triadFallbackUsed = false;

  // Most recent IMU reading (body frame), exposed for telemetry/UI --
  // computeControl() only ever reads the estimator/rate above, not these
  // directly.
  glm::vec3 lastGyroBody{0.0f};  // rad/s
  glm::vec3 lastAccelBody{0.0f}; // m/s^2 (specific force)

  // Hardware references
  RigidBody *body = nullptr;
  std::vector<ReactionWheel *> wheels;
  std::vector<Magnetorquer *> magnetorquers;
  IMU *imu = nullptr;
  Magnetometer *magnetometer = nullptr;
  StarTracker *starTracker = nullptr;

  // Ambient gravity the body is in, needed to interpret the IMU's
  // accelerometer as specific force (see IMU::sample). The cubesat
  // scenarios run at zero gravity, but ADCS doesn't hardcode that.
  glm::vec3 gravity{0.0f};

  // Ambient magnetic field (world frame, Tesla) at the body's current
  // location, needed to interpret the magnetometer (see Magnetometer::
  // sample) -- set externally by the scenario each frame from a
  // MagneticField model, same role `gravity` plays for the IMU. This
  // does NOT drive the magnetorquers themselves (they read their own
  // Magnetorquer::ambientFieldWorld, set the same way); it's only used
  // here to simulate what the magnetometer reads.
  glm::vec3 ambientFieldWorld{0.0f};

  // Most recent magnetometer reading and its time derivative (body frame,
  // Tesla / Tesla-per-second) -- exposed for telemetry/UI, and what the
  // B-dot law is computed from.
  glm::vec3 magFieldBody{0.0f};
  glm::vec3 magFieldRateBody{0.0f};

  // Absolute-attitude aiding: propagateEstimator()/correctEstimator()
  // implement a standard multiplicative EKF (state = [attitude error,
  // gyro bias error], propagated via strapdown quaternion kinematics,
  // corrected via a Kalman update against whichever absolute measurement
  // is available this cycle). The star tracker is the primary
  // measurement when it has a valid reading; when it doesn't (sun-
  // blinded, slewing too fast -- see StarTracker.h), this falls back to
  // the same two-vector (TRIAD) solve from the magnetometer + a coarse
  // sun-direction reading used before the EKF existed, just fed through
  // the same Kalman correction instead of a fixed-gain slerp blend, so
  // its (much larger) uncertainty is weighted correctly relative to the
  // star tracker's.
  //
  // 1-sigma noise the coarse sun-direction "sensor" adds when forming the
  // TRIAD fallback measurement -- not a dedicated SunSensor class, since
  // this fallback path doesn't need more fidelity than that.
  float sunSensorNoiseRad = glm::radians(0.5f);

  // Momentum desaturation ("momentum dumping"/"unloading"): reaction
  // wheels are purely internal actuators -- spinning one up and getting
  // the reaction torque back only redistributes momentum between body and
  // wheel, it can never remove momentum from the system. A wheel fighting
  // a secular disturbance keeps absorbing momentum in the same direction
  // until it hits maxSpeed and loses control authority there. The only
  // way to actually remove momentum is an *external* torque, which is
  // what the magnetorquers give access to (reacting against Earth's
  // field, not against the spacecraft itself).
  //
  // Unlike DETUMBLE, this deliberately is NOT a PointingMode: you want to
  // keep pointing correctly while bleeding down wheel momentum in the
  // background, not choose between the two. It runs every run() cycle
  // (see updateDesaturation()), concurrently with whatever mode/
  // controller is driving the wheels, using the classic cross-product law
  // (Stickler & Alfriend): m = -(k / |B|^2) * (H_wheel x B). Only
  // meaningful outside DETUMBLE, which already owns the magnetorquers for
  // B-dot -- running both at once would just have the two laws fighting
  // over the same hardware.
  bool desatAutoTriggerEnabled = true; // continuously monitor wheel saturation and start a pass when needed
  bool desatActive = false;            // true while a pass (auto- or manually-triggered) is in progress
  float desatTriggerSaturation = 0.8f; // max |wheel speed / maxSpeed| that starts a pass
  float desatStopSaturation = 0.3f;    // ...and the (lower, hysteresis) level a pass ends at
  float desatGain = 0.0f;              // tuned at construction from the magnetorquers' max moment

  // Force-starts a desaturation pass regardless of current wheel
  // saturation -- what a UI panel's "Desaturate Wheels Now" button calls.
  // Runs until desatStopSaturation is reached, same as an auto-triggered
  // pass; harmless (and simply does nothing) with no wheels/magnetorquers
  // or during DETUMBLE.
  void requestDesaturation() { desatActive = true; }

public:
  ADCS() = default;
  ADCS(RigidBody *body_, const std::vector<ReactionWheel *> &wheels_, IMU *imu_,
      const std::vector<Magnetorquer *> &magnetorquers_ = {}, Magnetometer *magnetometer_ = nullptr,
      StarTracker *starTracker_ = nullptr);
  ~ADCS();

  void run(float dt);
  void resetController();

  void computeGuidance(float dt);
  void computeControl(glm::quat attitude, glm::vec3 rate, float dt);
  void allocateActuators();
  void sendCommands();

  // Direct access to each controller's gains, for UI panels that want to
  // display/edit them live -- computeControl() reads these same fields
  // every call, so an edit here takes effect immediately, no separate
  // "apply" step needed.
  PIDController &pidController() { return pid; }
  LQRController &lqrController() { return lqr; }
  CascadedController &cascadedController() { return cascaded; }

  // Applies the per-mode gain/rate-limit preset (see ADCS.cpp) to whichever
  // controller is active, overwriting any manually-edited gains. Called
  // automatically from run() when `mode` changes; exposed publicly too so a
  // UI panel can offer an explicit "reset to auto-tuned" action.
  void retuneForMode();

private:
  PIDController pid;
  LQRController lqr;
  CascadedController cascaded;

  PointingMode lastTunedMode; // re-tune only when mode actually changes
  float detumbleKd = 0.0f;    // rate-damping gain for DETUMBLE, derived from inertia at construction

  bool hasPrevMagField = false;
  glm::vec3 prevMagFieldBody{0.0f};

  glm::vec3 bdotDipoleCommandBody{0.0f};  // desired net dipole moment (body frame) from B-dot, before per-rod allocation
  glm::vec3 desatDipoleCommandBody{0.0f}; // ...and from desaturation -- allocateActuators() sums both

  // mutable: computeTriadFallback() is logically const (it just builds a
  // candidate measurement, doesn't touch estimator state), but advancing
  // the RNG is an intentional side effect of sampling noise, not a
  // violation of that constness.
  mutable std::mt19937 sunSensorRng{std::random_device{}()};

  // Multiplicative EKF covariance, as four 3x3 blocks instead of one 6x6
  // matrix (GLM has no fixed-size 6-vector/6x6-matrix type, and the block
  // form is also just easier to read/verify against the hand-derived
  // update equations in ADCS.cpp): error state = [attitude error (3),
  // gyro bias error (3)]. covAA/covBB are the diagonal (attitude-
  // attitude, bias-bias) blocks; covAB is the attitude-bias cross-
  // covariance -- the transpose block (covBA) is never stored separately,
  // since P is symmetric by construction and every update preserves that.
  glm::mat3 covAA{1.0f};
  glm::mat3 covAB{0.0f};
  glm::mat3 covBB{1.0f};

  // Continuous-time process noise spectral densities (isotropic, so
  // scalars rather than full 3x3 matrices): how fast attitude uncertainty
  // grows between corrections from gyro white noise, and how fast bias
  // uncertainty grows from bias random walk. Derived from the IMU's own
  // noise model at construction -- see ADCS.cpp.
  float gyroNoisePsd = 0.0f;
  float gyroBiasWalkPsd = 0.0f;

  // DETUMBLE's control laws: pure rate damping via wheels, or the B-dot law
  // via magnetorquers -- see DetumbleActuator.
  glm::vec3 computeDetumbleTorque(const glm::vec3 &rate) const;
  glm::vec3 computeBdotDipoleCommand() const;

  // Updates desatActive (auto-trigger/auto-stop against wheel saturation)
  // and desatDipoleCommandBody (the cross-product law) -- see
  // desatAutoTriggerEnabled's comment above. Called once per run() cycle.
  void updateDesaturation();

  // EKF "predict" step: strapdown-integrates estimatedAttitude using the
  // bias-corrected gyro reading, and propagates the covariance blocks
  // through the linearized attitude+bias error dynamics
  // (d(deltaTheta)/dt = -omega x deltaTheta - deltaBias, d(deltaBias)/dt = 0),
  // first-order (Euler) discretized -- adequate at this sim's ~20 Hz ADCS
  // rate without needing a closed-form or Van Loan discretization.
  void propagateEstimator(const glm::vec3 &gyroMeasured, float dt);

  // EKF "correct" step: standard multiplicative-EKF Kalman update against
  // an absolute attitude measurement qMeas with isotropic covariance R
  // (rad^2) -- shared by the star tracker path and the TRIAD fallback
  // (see run()), since both are ultimately "here's an absolute attitude
  // reading, here's how much to trust it," just with very different R.
  void correctEstimator(const glm::quat &qMeas, float R);

  // Builds the TRIAD-derived fallback measurement (see computeTriadAttitude
  // and sunSensorNoiseRad's comment) used when the star tracker has no
  // valid reading this cycle. Returns false if no usable measurement can
  // be formed (no sun position configured, or the two references are too
  // close to parallel for a reliable TRIAD solve).
  bool computeTriadFallback(glm::quat &outAttitude, float &outR) const;

  // Two-vector (TRIAD) deterministic attitude solve: given a primary and
  // secondary direction, each measured in BODY frame and known in REF
  // (world) frame, returns the orientation quaternion q such that
  // q*bodyVec ~= refVec for both vectors (exactly for the primary,
  // as-close-as-orthogonality-allows for the secondary).
  static glm::quat computeTriadAttitude(const glm::vec3 &primaryBody, const glm::vec3 &primaryRef,
                                        const glm::vec3 &secondaryBody, const glm::vec3 &secondaryRef);

  // Minimum-norm allocation of a commanded 3-vector across a set of
  // actuator axes via the Moore-Penrose pseudoinverse -- shared by
  // allocateActuators() for both the wheel cluster and the magnetorquer
  // cluster, since it's the identical A+ = A^T(AA^T)^-1 math either way,
  // just with a different set of body-frame axes.
  static std::vector<float> allocateViaPseudoinverse(
      const glm::vec3 &command, const std::vector<glm::vec3> &axesBody);
};
