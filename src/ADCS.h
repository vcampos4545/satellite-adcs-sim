#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include "Controllers.h"
#include "FlightTypes.h"
#include "FDIR.h"

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

// Flight software for a 3-axis-stabilized cubesat: attitude estimation
// (multiplicative EKF, star tracker primary / sun+magnetometer TRIAD
// fallback), guidance (six pointing modes), attitude control (PID/LQR/
// cascaded, auto-tuned from bus inertia), B-dot detumble, and cross-
// product-law momentum desaturation.
//
// Hardware-abstracted by design: this class never references RigidBody,
// PhysicsWorld, or any sensor/actuator simulation type (see FlightTypes.h)
// -- configure() takes a plain HardwareConfig, step() takes plain
// FSWInputs and returns plain FSWOutputs. Whatever drives it (this
// simulation's harness today, a HIL rig or real flight hardware later)
// owns all sensor sampling and actuator command application; this class
// only ever sees data, never simulation objects.
class ADCS
{
public:
  // The commanded mode -- what ground/UI last asked for. Guidance/control
  // do NOT read this directly; see `effectiveMode` below.
  PointingMode mode = PointingMode::TARGET;
  ControllerType controllerType = ControllerType::PID;
  DetumbleActuator detumbleActuator = DetumbleActuator::REACTION_WHEELS;

  // Autonomous mode manager / fault monitor: runs every step() cycle and
  // may override `mode` with a safe mode of its own choosing (see FDIR.h)
  // -- the same "mode manager" vs. "GNC" split a real FSW's task structure
  // has. Public so a UI panel can show its state/event log and tune
  // thresholds live, same pattern as pidController()/lqrController() below.
  FDIR fdir;

  // What guidance/control actually execute this cycle -- `mode` unless
  // `fdir` is in SAFE_HOLD and has overridden it. Read-only in spirit
  // (step() recomputes it every cycle from `fdir.evaluate()`); exposed so
  // a UI panel can show "commanded X, but actually flying Y because of a
  // fault" instead of that distinction being invisible.
  PointingMode effectiveMode = PointingMode::TARGET;

  glm::vec3 target = {0.5f, 0.5f, 0.5f};      // world position for TARGET/SLEW/FINE_POINTING
  glm::vec3 sunPosition = {0.0f, 0.0f, 0.0f}; // world position for SUN_POINTING

  // World-frame ambient magnetic field, needed as TRIAD's reference vector
  // and for the B-dot/desaturation laws' frame-consistency -- a real
  // system would get this from an onboard field model (e.g. IGRF
  // coefficients evaluated against a nav solution), the same category of
  // "known reference" `target`/`sunPosition` already are, not a live
  // sensor reading. Set by the harness each cycle from its MagneticField
  // model.
  glm::vec3 ambientFieldWorld{0.0f};

  glm::quat targetAttitude;

  // Pointing error (degrees) against targetAttitude, updated by
  // computeGuidance() -- held at its last value during DETUMBLE, which has
  // no attitude target. This is what the FSW itself actually computes and
  // could act on (against estimatedAttitude); there is deliberately no
  // ground-truth equivalent here anymore -- this class has no way to know
  // ground truth at all now, which is the point. (A harness that wants a
  // true-vs-estimated comparison for its own diagnostics computes it
  // itself, using its own direct access to the simulation.)
  float estimatedPointingErrorDeg = 0.0f;

  glm::vec3 torqueCommand;
  std::array<float, NUM_WHEELS> wheelCommands{};        // wheelCommands[i] -> torque for wheel i (Nm)
  std::array<float, NUM_TORQUERS> magnetorquerCommands{}; // magnetorquerCommands[i] -> dipole moment for torquer i (A*m^2)

  // B-dot gain (A*m^2 per T/s): m_cmd = -bdotGain * dB/dt. Tuned once at
  // configure() from the magnetorquers' max moment; exposed here so a UI
  // panel can retune it live.
  float bdotGain = 0.0f;

  // Manual actuator override: when true, step() skips guidance/control/
  // allocation entirely and sends manualWheelTorqueNm/
  // manualMagnetorquerMomentAm2 straight through -- for a UI panel that
  // wants direct per-actuator sliders instead of going through a pointing
  // mode.
  bool manualOverride = false;
  std::array<float, NUM_WHEELS> manualWheelTorqueNm{};
  std::array<float, NUM_TORQUERS> manualMagnetorquerMomentAm2{};

  // Attitude estimate, propagated by strapdown-integrating the (bias-
  // corrected) gyro reading every step() and periodically corrected
  // against an absolute reference -- see propagateEstimator()/
  // correctEstimator() and the multiplicative-EKF comment below.
  glm::quat estimatedAttitude{1, 0, 0, 0};

  // Gyro bias estimate (rad/s, body frame) -- the EKF's second state
  // besides attitude. Subtracted from every raw gyro reading before it's
  // used for propagation *or* rate feedback to the attitude controller
  // (see step()): a rate loop that nulls the raw, still-biased measurement
  // settles at a true rate equal to minus that bias, a real steady-state
  // pointing error tuning alone can't fix.
  glm::vec3 gyroBiasEstimate{0.0f};

  // 1-sigma attitude uncertainty (degrees), derived from the EKF
  // covariance's attitude block -- exposed so FSW/UI can judge how much
  // to trust the estimate right now, not just read a point value with no
  // sense of its own confidence. Grows during star-tracker dropouts
  // (sun-blinded, slewing too fast) and shrinks again once a correction
  // lands.
  float attitudeUncertaintyDeg = 0.0f;

  // Whether the star tracker had a valid reading this cycle (vs. blinded
  // by the sun or slewing too fast) and, if not, whether the TRIAD
  // fallback supplied a correction instead -- both are things the FSW
  // itself genuinely knows (not ground truth), exposed for telemetry/UI.
  bool starTrackerValid = false;
  bool triadFallbackUsed = false;

  // Most recent IMU reading (body frame), exposed for telemetry/UI --
  // computeControl() only ever reads the estimator/rate above, not these
  // directly.
  glm::vec3 lastGyroBody{0.0f};  // rad/s
  glm::vec3 lastAccelBody{0.0f}; // m/s^2 (specific force)

  // Most recent magnetometer reading and its time derivative (body frame,
  // Tesla / Tesla-per-second) -- exposed for telemetry/UI, and what the
  // B-dot law is computed from.
  glm::vec3 magFieldBody{0.0f};
  glm::vec3 magFieldRateBody{0.0f};

  // Most recent EPS reading -- exposed for telemetry/UI (and what fdir's
  // low-battery fault is computed from, see step()).
  float batterySoc = 1.0f;
  float batteryVoltageV = 0.0f;

  // 1-sigma sun-sensor angular uncertainty (radians) this class assumes
  // when weighting the TRIAD fallback measurement -- a configured sensor
  // spec (matching the harness's SunSensor default), not something read
  // live off a sensor object.
  float sunSensorNoiseRad = glm::radians(0.5f);

  // Momentum desaturation ("momentum dumping"/"unloading"): reaction
  // wheels are purely internal actuators -- spinning one up and getting
  // the reaction torque back only redistributes momentum between body and
  // wheel, it can never remove momentum from the system. The only way to
  // actually remove momentum is an *external* torque, which is what the
  // magnetorquers give access to.
  //
  // Unlike DETUMBLE, this deliberately is NOT a PointingMode: you want to
  // keep pointing correctly while bleeding down wheel momentum in the
  // background, not choose between the two. It runs every step() cycle,
  // concurrently with whatever mode/controller is driving the wheels,
  // using the classic cross-product law (Stickler & Alfriend):
  // m = (k / |B|^2) * (H_wheel x B). Only meaningful outside DETUMBLE,
  // which already owns the magnetorquers for B-dot.
  bool desatAutoTriggerEnabled = true; // continuously monitor wheel saturation and start a pass when needed
  bool desatActive = false;            // true while a pass (auto- or manually-triggered) is in progress
  float desatTriggerSaturation = 0.8f; // max |wheel speed / maxSpeed| that starts a pass
  float desatStopSaturation = 0.3f;    // ...and the (lower, hysteresis) level a pass ends at
  float desatGain = 0.0f;              // tuned at configure() from the magnetorquers' max moment

  // Force-starts a desaturation pass regardless of current wheel
  // saturation -- what a UI panel's "Desaturate Wheels Now" button calls.
  void requestDesaturation() { desatActive = true; }

public:
  ADCS() = default;

  // Sets the fixed hardware configuration (wheel/torquer geometry and
  // limits, bus inertia) and the initial attitude estimate -- everything a
  // real flight computer would get from a compile-time hardware config
  // table plus a cold-start attitude acquisition, rather than reaching
  // into a RigidBody. Call once before the first step().
  void configure(const HardwareConfig &hw, const glm::quat &initialAttitude);

  void resetController();

  // The FSW cycle: sensor data in, actuator commands out. Pure function of
  // (current state, in, dt) -- no knowledge of where `in` came from.
  FSWOutputs step(const FSWInputs &in, float dt);

  // Direct access to each controller's gains, for UI panels that want to
  // display/edit them live -- computeControl() reads these same fields
  // every call, so an edit here takes effect immediately, no separate
  // "apply" step needed.
  PIDController &pidController() { return pid; }
  LQRController &lqrController() { return lqr; }
  CascadedController &cascadedController() { return cascaded; }

  // Applies the per-mode gain/rate-limit preset (see ADCS.cpp) to whichever
  // controller is active, overwriting any manually-edited gains. Called
  // automatically from step() when `mode` changes; exposed publicly too so
  // a UI panel can offer an explicit "reset to auto-tuned" action.
  void retuneForMode();

private:
  HardwareConfig hw_;

  PIDController pid;
  LQRController lqr;
  CascadedController cascaded;

  PointingMode lastTunedMode; // re-tune only when mode actually changes
  float detumbleKd = 0.0f;    // rate-damping gain for DETUMBLE, derived from bus inertia at configure()

  bool hasPrevMagField = false;
  glm::vec3 prevMagFieldBody{0.0f};

  glm::vec3 bdotDipoleCommandBody{0.0f};  // desired net dipole moment (body frame) from B-dot, before per-rod allocation
  glm::vec3 desatDipoleCommandBody{0.0f}; // ...and from desaturation -- allocateActuators() sums both

  // Multiplicative EKF covariance, as four 3x3 blocks instead of one 6x6
  // matrix (no fixed-size 6x6 type in glm, and the block form is easier to
  // read/verify against the hand-derived update equations in ADCS.cpp):
  // error state = [attitude error (3), gyro bias error (3)]. covAA/covBB
  // are the diagonal (attitude-attitude, bias-bias) blocks; covAB is the
  // attitude-bias cross-covariance -- the transpose block (covBA) is never
  // stored separately, since P is symmetric by construction and every
  // update preserves that.
  glm::mat3 covAA{1.0f};
  glm::mat3 covAB{0.0f};
  glm::mat3 covBB{1.0f};

  // Continuous-time process noise spectral densities (isotropic, so
  // scalars rather than full 3x3 matrices): how fast attitude uncertainty
  // grows between corrections from gyro white noise, and how fast bias
  // uncertainty grows from bias random walk. Set from a fixed, known gyro
  // spec at configure() -- see ADCS.cpp.
  float gyroNoisePsd = 0.0f;
  float gyroBiasWalkPsd = 0.0f;

  void computeGuidance(const glm::vec3 &spacecraftPositionWorld, float dt);
  void computeControl(glm::quat attitude, glm::vec3 rate, float dt);
  void updateDesaturation(const FSWInputs &in);
  void allocateActuators();
  FSWOutputs buildOutputs() const;

  // DETUMBLE's control laws: pure rate damping via wheels, or the B-dot law
  // via magnetorquers -- see DetumbleActuator.
  glm::vec3 computeDetumbleTorque(const glm::vec3 &rate) const;
  glm::vec3 computeBdotDipoleCommand() const;

  // EKF "predict" step: strapdown-integrates estimatedAttitude using the
  // bias-corrected gyro reading, and propagates the covariance blocks
  // through the linearized attitude+bias error dynamics
  // (d(deltaTheta)/dt = -omega x deltaTheta - deltaBias, d(deltaBias)/dt = 0),
  // first-order (Euler) discretized -- adequate at this sim's ~20 Hz ADCS
  // rate without needing a closed-form or Van Loan discretization.
  void propagateEstimator(const glm::vec3 &gyroMeasured, float dt);

  // EKF "correct" step: standard multiplicative-EKF Kalman update against
  // an absolute attitude measurement qMeas with isotropic covariance R
  // (rad^2) -- shared by the star tracker path and the TRIAD fallback,
  // since both are ultimately "here's an absolute attitude reading, here's
  // how much to trust it," just with very different R.
  void correctEstimator(const glm::quat &qMeas, float R);

  // Builds the TRIAD-derived fallback measurement used when the star
  // tracker has no valid reading this cycle. Returns false if no usable
  // measurement can be formed (magnetometer/sun-sensor invalid this cycle,
  // no sun position configured, or the two references are too close to
  // parallel for a reliable TRIAD solve).
  bool computeTriadFallback(const FSWInputs &in, glm::quat &outAttitude, float &outR) const;

  // Two-vector (TRIAD) deterministic attitude solve: given a primary and
  // secondary direction, each measured in BODY frame and known in REF
  // (world) frame, returns the orientation quaternion q such that
  // q*bodyVec ~= refVec for both vectors (exactly for the primary,
  // as-close-as-orthogonality-allows for the secondary).
  static glm::quat computeTriadAttitude(const glm::vec3 &primaryBody, const glm::vec3 &primaryRef,
                                        const glm::vec3 &secondaryBody, const glm::vec3 &secondaryRef);

  // Minimum-norm allocation of a commanded 3-vector across a fixed set of
  // N actuator axes via the Moore-Penrose pseudoinverse -- shared by
  // allocateActuators() for both the wheel cluster and the magnetorquer
  // cluster, since it's the identical A+ = A^T(AA^T)^-1 math either way,
  // just with a different (fixed, no heap allocation) set of body-frame
  // axes.
  template <int N>
  static std::array<float, N> allocateViaPseudoinverse(
      const glm::vec3 &command, const std::array<glm::vec3, N> &axesBody);
};
