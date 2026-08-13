#pragma once
#include <glm/glm.hpp>

// This project's specific simulated spacecraft: physical design (mass,
// geometry, actuator/sensor specs, battery), the EPS power-budget
// assumptions that follow from that hardware, and its onboard FSW/task
// timing. Everything here can change simulated behavior -- contrast
// VisualizationConfig.h (rendering/camera/UI knobs that never do) and
// PhysicalConstants.h (universal physics/astronomy, not specific to this
// spacecraft).
namespace SatelliteConfig
{
  // What AtmosphericDrag/SolarRadiationPressure (predicted orbit path,
  // ground-station pass prediction) model this spacecraft as -- a
  // simplified mass/cross-section pair rather than the full mirror+bus
  // breakdown above, since neither force model needs more than that.
  constexpr double SPACECRAFT_MASS_KG = 75.0;
  constexpr double SPACECRAFT_CROSS_SECTION_M2 = 18.0 * 18.0; // mirror's full face area

  // ---------------------------------------------------------------------
  // Structure: an 18m x 18m deployable mirror (the RigidBody itself) plus
  // a small bus core -- see docs/ALGORITHMS.md's "Spacecraft Structure"
  // section for the composite inertia tensor buildSatellite() derives
  // from these.
  // ---------------------------------------------------------------------
  constexpr float MIRROR_SPAN_M = 18.0f;
  constexpr float MIRROR_THICKNESS_M = 0.005f;
  constexpr float MIRROR_MASS_KG = 35.0f; // rounded up from an areal-density estimate for structure margin
  constexpr float BUS_CORE_SIDE_M = 0.5f;
  constexpr float BUS_MASS_KG = 40.0f;
  constexpr float TOTAL_MASS_KG = MIRROR_MASS_KG + BUS_MASS_KG;

  // ---------------------------------------------------------------------
  // Reaction wheel pyramid: 4 wheels, spin axes tilted WHEEL_PYRAMID_SKEW_DEG
  // from body +Z, spaced 90 deg apart in azimuth, mounted near the +Z face
  // -- see buildSatellite()'s own comment for why (matches how a real RWA
  // pyramid bracket is bolted to one panel).
  // ---------------------------------------------------------------------
  constexpr float WHEEL_PYRAMID_SKEW_DEG = 45.0f;
  constexpr float WHEEL_MOUNT_RADIUS_M = 0.15f;
  constexpr float WHEEL_MOUNT_HEIGHT_M = 0.2f;
  constexpr float WHEEL_MAX_TORQUE_NM = 0.2f;
  constexpr float WHEEL_MAX_SPEED_RPM = 6000.0f;
  constexpr float WHEEL_INERTIA_KGM2 = 1.6e-3f;

  // ---------------------------------------------------------------------
  // Magnetorquers: 3 mutually orthogonal rods along the body axes -- see
  // buildSatellite()'s own comment on why no tilt (unlike the wheels, a
  // torque rod has no momentum to distribute across axes).
  // ---------------------------------------------------------------------
  constexpr float TORQUER_MOUNT_OFFSET_M = 0.2f;
  constexpr float TORQUER_MAX_MOMENT_AM2 = 15.0f; // matches a real torque-rod product in this bus class (e.g. ZARM Technik MT15-1)

  // ---------------------------------------------------------------------
  // Sensor mount positions (body frame) -- off-center like a real PCB, so
  // the IMU's accelerometer isn't trivially always-zero (it picks up
  // centripetal/tangential terms from body rotation), and away from the
  // torque rods/wheels for the magnetometer (their fields would otherwise
  // swamp it).
  // ---------------------------------------------------------------------
  const glm::vec3 IMU_MOUNT_POS{0.15f, 0.15f, 0.1f};
  const glm::vec3 MAGNETOMETER_MOUNT_POS{-0.15f, -0.15f, 0.1f};

  // ---------------------------------------------------------------------
  // Solar panels: one body-mounted cell array per face (vs. a single
  // sun-tracking array) -- see buildSatellite()'s own comment.
  // ---------------------------------------------------------------------
  constexpr float SOLAR_PANEL_AREA_M2 = 1.0f;     // one representative 1.0m x 1.0m deployed panel face
  constexpr float SOLAR_PANEL_EFFICIENCY = 0.28f; // representative triple-junction cell

  // ---------------------------------------------------------------------
  // Battery: a representative Li-ion pack sized for this bus.
  // ---------------------------------------------------------------------
  constexpr float BATTERY_CAPACITY_WH = 40.0f;
  constexpr float BATTERY_MIN_VOLTAGE_V = 6.0f;
  constexpr float BATTERY_MAX_VOLTAGE_V = 8.4f;
  constexpr float BATTERY_INITIAL_SOC = 0.8f;

  // ---------------------------------------------------------------------
  // EPS power budget: representative wattages for every load on the bus,
  // summed each FSW cycle against solar generation -- see
  // Satellite::updatePower() and docs/ALGORITHMS.md's "EPS" section.
  // ---------------------------------------------------------------------
  constexpr float POWER_OBC_BASELINE_W = 0.4f; // onboard computer + FSW housekeeping, always on
  constexpr float POWER_IMU_W = 0.05f;
  constexpr float POWER_MAGNETOMETER_W = 0.03f;
  constexpr float POWER_STAR_TRACKER_W = 0.35f; // star trackers are the power-hungriest sensor on a Satellite bus
  constexpr float POWER_SUN_SENSOR_W = 0.01f;

  // Reaction wheel: a small standby/electronics draw per wheel regardless
  // of command, plus mechanical power (torque * speed) over an assumed
  // motor efficiency -- the same "idle + effort-proportional" shape a real
  // motor draws.
  constexpr float WHEEL_IDLE_POWER_W = 0.02f;
  constexpr float WHEEL_MOTOR_EFFICIENCY = 0.6f;

  // Magnetorquer: idle electronics draw plus a resistive coil term
  // proportional to the commanded dipole moment (a real rod's power is
  // I^2*R, i.e. proportional to moment^2, but a linear approximation is
  // close enough at these scales).
  constexpr float TORQUER_IDLE_POWER_W = 0.01f;
  constexpr float TORQUER_POWER_PER_AM2_W = 0.5f;

  // ---------------------------------------------------------------------
  // FSW/task timing: this spacecraft's onboard cyclic-executive rates --
  // see FlightSoftware.h's own header comment for the reasoning behind
  // each sensor's sample period (IMU/magnetometer/wheel telemetry aren't
  // listed here since they're sampled fresh every TIME_STEP_S cycle; only
  // the genuinely slower sensors below need their own throttled schedule).
  // ---------------------------------------------------------------------
  constexpr float TIME_STEP_S = 0.05f;                 // nominal FSW cycle period (20 Hz)
  constexpr float STAR_TRACKER_SAMPLE_PERIOD_S = 0.2f; // 5 Hz -- image-processing-based attitude solve
  constexpr float SUN_SENSOR_SAMPLE_PERIOD_S = 0.1f;   // 10 Hz -- coarse analog sensor polled over a bus
  constexpr float POWER_SAMPLE_PERIOD_S = 1.0f;        // 1 Hz -- typical I2C fuel-gauge IC polling rate

  // Caps how much simulated backlog (SimControls::timeScale-scaled real
  // elapsed time) main()'s fixed-step accumulator will process in a single
  // render frame, the same "spiral of death" clamp PhysicsWorld::step()
  // already applies to its own inner accumulator. Without this, a high
  // timeScale (or a real stall) would demand hundreds of catch-up
  // FSW/physics steps in one frame, freezing rendering until it worked
  // through the backlog.
  constexpr float FSW_TIMER_MAX_S = 1.0f; // at most 1s / TIME_STEP_S = 20 FSW steps per render frame

  // ---------------------------------------------------------------------
  // Ground targeting / disturbances
  // ---------------------------------------------------------------------
  constexpr float GROUND_STATION_MIN_ELEVATION_DEG = 10.0f; // minimum usable RF link elevation for target selection
  constexpr float TUMBLE_KICK_RAD_S = 1.5f;                 // default magnitude for the Simulation tab's "Kick into random tumble" control
}
