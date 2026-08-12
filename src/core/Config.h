#pragma once
#include <glm/glm.hpp>

// ---------------------------------------------------------------------------
// Config
// ---------------------------------------------------------------------------
namespace Config
{
  // Visualization
  constexpr float SATELLITE_VISUAL_SCALE = 550.0f;
  constexpr float SCENE_AMBIENT_LIGHT = 0.35f;
  constexpr float TARGET_MARKER_RADIUS_M = 5.0e4f;

  // Simulation camera
  constexpr float CAMERA_NEAR = 10.0f;
  constexpr float CAMERA_FAR = 2.0e11f;
  constexpr float CAMERA_FOV = 45.0f;
  constexpr float SATELLITE_HALF_DIAGONAL_M = 12.7279f; // 0.5*sqrt(18^2 + 18^2), real (pre-scale) mirror half-diagonal
  constexpr float CAMERA_MIN_DISTANCE = SATELLITE_HALF_DIAGONAL_M * SATELLITE_VISUAL_SCALE * 2.0f;
  constexpr float CAMERA_INITIAL_DISTANCE = CAMERA_MIN_DISTANCE * 1.6f;
  constexpr float CAMERA_MAX_DISTANCE = 1.0e8f;
  constexpr float ZOOM_SENSITIVITY = 1.0f;
  constexpr float PAN_SENSITIVITY = 0.2f;

  constexpr float FOOTPRINT_MIN_ELEVATION_DEG = 0.0f;
  constexpr float GROUND_STATION_MIN_ELEVATION_DEG = 10.0f;

  // Ground-station pass prediction (predictGroundStationPasses): how far
  // ahead to search, and the fixed time step the AOS/LOS/max-elevation
  // search advances by. A LEO pass typically lasts 5-15 minutes, so a 15s
  // step gives tens of samples per pass (AOS/LOS timing resolution is
  // +/- one step) without making a 24h search expensive. Recomputed on a
  // real (wall-clock, not simulated) timer -- see
  // PASS_PREDICTION_REFRESH_S -- so cost stays constant regardless of
  // SimControls::timeScale, unlike the orbit path's own simDt-driven
  // refresh.
  constexpr double PASS_PREDICTION_LOOKAHEAD_S = 24.0 * 3600.0;
  constexpr double PASS_PREDICTION_STEP_S = 15.0;
  constexpr float PASS_PREDICTION_REFRESH_S = 30.0f;

  // Ground track: how much history to keep and how often to sample it.
  // Sampling once every GROUND_TRACK_SAMPLE_INTERVAL_S of mission time
  // (not every frame) at GROUND_TRACK_MAX_POINTS keeps the trail spanning
  // multiple orbits without growing unbounded over a long-running mission.
  constexpr int GROUND_TRACK_MAX_POINTS = 400;
  constexpr float GROUND_TRACK_SAMPLE_INTERVAL_S = 15.0f;

  constexpr float TUMBLE_KICK_RAD_S = 1.5f;

  // Nominal FSW cycle period (20 Hz) -- the single shared rate orbit
  // propagation, PhysicsWorld::step(), and one FlightSoftware::step() cycle
  // all advance by together, once per main()-loop iteration (see its own
  // fixed-step accumulator).
  constexpr float TIME_STEP_S = 0.05f;

  // Per-sensor sample periods -- FlightSoftware::step() samples each
  // sensor independently at its own realistic firmware rate rather than
  // synchronized to the FSW cycle above (see its own header comment).
  // IMU/magnetometer/wheel telemetry aren't listed here: real MEMS
  // gyros/accelerometers, magnetometers, and motor-controller telemetry
  // all comfortably exceed this loop's 20 Hz on their own, so sampling
  // them fresh every cycle already matches real hardware; only the
  // genuinely slower sensors below need their own throttled schedule.
  constexpr float STAR_TRACKER_SAMPLE_PERIOD_S = 0.2f; // 5 Hz -- image-processing-based attitude solve
  constexpr float SUN_SENSOR_SAMPLE_PERIOD_S = 0.1f;   // 10 Hz -- coarse analog sensor polled over a bus
  constexpr float POWER_SAMPLE_PERIOD_S = 1.0f;        // 1 Hz -- typical I2C fuel-gauge IC polling rate

  constexpr int TELEMETRY_HISTORY_SAMPLES = 300;

  // Satellite
  constexpr double SPACECRAFT_MASS_KG = 75.0;
  constexpr double SPACECRAFT_CROSS_SECTION_M2 = 18.0 * 18.0; // mirror's full face area

  // Global dipole field-line visualization (see traceDipoleFieldLines):
  // seed colatitudes/azimuths (both hemispheres) for the traced loops, an
  // RK4 arclength step size, and a per-line point cap as a safety bound
  // against near-axis seeds (whose loops can be very large) never
  // reaching the closure/max-radius stopping condition.
  constexpr float FIELD_LINE_COLATITUDES_DEG[] = {15.0f, 30.0f, 45.0f};
  constexpr int FIELD_LINE_AZIMUTH_COUNT = 10;
  constexpr float FIELD_LINE_STEP_FRAC_EARTH_RADIUS = 0.03f; // RK4 step, as a fraction of Earth's radius
  constexpr int FIELD_LINE_MAX_POINTS = 200;
  constexpr float FIELD_LINE_MAX_RADIUS_FRAC_EARTH_RADIUS = 12.0f; // safety cutoff for open-looking loops

  // The real RigidBody *is* the 18m x 18m mirror plate now (see
  // buildSatellitePyramid()), so its own +Z face is the actual reflecting
  // surface -- MIRROR_NORMAL_BODY still names that axis for
  // drawSunReflection()'s geometry. BUS_SIZE is purely a decorative box
  // mounted just behind the plate (-Z), representing the small bus core
  // sitting at the mirror's center -- not a second RigidBody, not part of
  // physics, same "visualization only" role the old decorative mirror
  // box used to play before the real body became the mirror itself.
  const glm::vec3 BUS_SIZE{0.5f, 0.5f, 0.5f};
  const glm::vec3 MIRROR_NORMAL_BODY{0.0f, 0.0f, 1.0f};
  constexpr float REFLECTED_RAY_LENGTH = 1.0f;

  // Orbit visualization: PhysicsWorld's own coordinate frame *is* ECI
  // here -- Earth's center is the world origin (real meters), and
  // sat.body->position is bridged from orbitState every frame (see
  // main()), so there's no separate render scale/offset to track
  // anymore: Earth is drawn at its real radius, at the real origin, and
  // the satellite is wherever its real orbital position puts it.
  constexpr int ORBIT_PATH_POINTS = 120;       // segments in the predicted-path polyline
  constexpr float ORBIT_PATH_REFRESH_S = 5.0f; // real seconds between path recomputes

  // EPS (electrical power subsystem) power budget: representative
  // wattages for every load on the bus, summed each ADCS cycle against
  // what the solar panels generate that same cycle -- see the FLIGHT
  // SOFTWARE block in main(). Generation is gated by EclipseModel::
  // inEclipse() against the real orbital position (see rigidbody/orbit/
  // EclipseModel.h) -- cylindrical shadow only, not the true conical
  // umbra/penumbra, a documented simplification (see docs/ALGORITHMS.md).
  constexpr float SOLAR_FLUX_WM2 = 1361.0f; // solar constant at 1 AU

  constexpr float POWER_OBC_BASELINE_W = 0.4f; // onboard computer + FSW housekeeping, always on
  constexpr float POWER_IMU_W = 0.05f;
  constexpr float POWER_MAGNETOMETER_W = 0.03f;
  constexpr float POWER_STAR_TRACKER_W = 0.35f; // star trackers are the power-hungriest sensor on a Satellite bus
  constexpr float POWER_SUN_SENSOR_W = 0.01f;

  // Reaction wheel: a small standby/electronics draw per wheel regardless
  // of command, plus mechanical power (torque * speed) over an assumed
  // motor efficiency -- the same "idle + effort-proportional" shape a real
  // motor draws, not a flat number that ignores what the wheel is actually
  // being asked to do.
  constexpr float WHEEL_IDLE_POWER_W = 0.02f;
  constexpr float WHEEL_MOTOR_EFFICIENCY = 0.6f;

  // Magnetorquer: idle electronics draw plus a resistive coil term
  // proportional to the commanded dipole moment (a real rod's power is
  // I^2*R, i.e. proportional to moment^2, but a linear approximation is
  // close enough at these scales and avoids implying more precision about
  // the coil's actual resistance than this project has any basis for).
  constexpr float TORQUER_IDLE_POWER_W = 0.01f;
  constexpr float TORQUER_POWER_PER_AM2_W = 0.5f;

  // Celestial body constants
  constexpr double GM_SUN = 1.32712440018e20;
  constexpr double GM_MOON = 4.9048695e12;
}
