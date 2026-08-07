#pragma once
#include <glm/glm.hpp>

// ---------------------------------------------------------------------------
// Config
// ---------------------------------------------------------------------------
namespace Config
{
  // Satellite is a real 0.1m cubesat at a real ~6.9e6m orbital radius --
  // rendered at its actual size, it would be a sub-pixel dot from any
  // camera distance that also shows Earth. SATELLITE_VISUAL_SCALE
  // inflates every *drawn* dimension of the satellite (wireframe/wheels/
  // rods/arrows/mirror/grid/field arrows) around its real position, same
  // "not to scale but at the right place" idea already used for the sun
  // marker/B-field arrows -- it never touches the physical quantities
  // (RigidBody size, wheel inertia, mount offsets used for actual
  // dynamics) any of the FSW/physics math is computed from, only what
  // gets handed to the renderer. Applied as: draw the true local geometry
  // scaled up around the satellite's real center, i.e.
  // `satPos + (trueOffset) * SATELLITE_VISUAL_SCALE` for anything offset
  // from center, and `trueSize * SATELLITE_VISUAL_SCALE` for anything
  // sized -- see drawSatelliteWireframe/drawReactionWheels/etc.
  constexpr float SATELLITE_VISUAL_SCALE = 100000.0f;

  // Camera: follows the satellite's real ECI position (see main()'s
  // orbit.setTarget() call each frame), so distances are tuned for a
  // scene that spans everything from the now-visually-inflated satellite
  // up to a ~6.9e6m orbital radius in the same world.
  //
  // MIN/INITIAL_DISTANCE are derived from SATELLITE_VISUAL_SCALE rather
  // than independent constants, so they can't drift out of sync with it
  // again if that scale ever changes: the satellite's own drawn geometry
  // reaches SATELLITE_VISUAL_SCALE * 0.25 from its center (the body-axis
  // arrows, drawSatelliteWireframe's largest extent), so MIN_DISTANCE
  // (0.5x the full scale, 2x the arrows' own reach) keeps the camera from
  // ending up inside the satellite's inflated geometry when fully zoomed
  // in.
  //
  // MAX_DISTANCE must stay well *inside* CAMERA_FAR: OrbitalCamera clamps
  // `distance` to [MIN_DISTANCE, MAX_DISTANCE], and if that range extends
  // past the far clip plane, zooming out toward MAX_DISTANCE pushes the
  // camera's target (and the whole scene around it) outside the far
  // plane's render range, which just makes everything vanish -- not what
  // "zoom out" should do. Earth's orbit diameter is ~1.37e7m; 1e8
  // (~100,000 km, ~7x that) comfortably frames the whole Earth+orbit
  // system with room to spare, and CAMERA_FAR sits a further 5x beyond
  // that so the camera is never anywhere near clipping its own target.
  // The near:far ratio this creates (~5e7) needs the logarithmic depth
  // buffer enabled in main() (gui.setLogDepth) to avoid z-fighting a
  // standard depth buffer can't handle at this range.
  // CAMERA_FAR must clear the real Sun distance (~1.496e11m, 1 AU) with
  // margin, now that the Sun/Moon are drawn at their real ECI positions
  // (see main()'s adcs.sunPosition / moonPositionEci) rather than a
  // scaled-for-visibility stand-in -- otherwise both are silently clipped
  // by the far plane and never rendered at all.
  constexpr float CAMERA_NEAR = 10.0f;
  constexpr float CAMERA_FAR = 2.0e11f;
  constexpr float CAMERA_FOV = 45.0f;
  constexpr float CAMERA_INITIAL_DISTANCE = SATELLITE_VISUAL_SCALE * 0.8f;
  constexpr float CAMERA_MIN_DISTANCE = SATELLITE_VISUAL_SCALE * 0.5f;
  constexpr float CAMERA_MAX_DISTANCE = 1.0e8f;
  constexpr float ZOOM_SENSITIVITY = 1.0f;
  constexpr float PAN_SENSITIVITY = 0.2f;

  // Scene ambient light (VGL's GUI::setAmbientLight -- see its own header
  // comment): the floor every lit surface's shading is added on top of,
  // regardless of light direction. VGL's own default (0.15) leaves
  // Earth's night side reading as near-black; this project's visual goal
  // is a softer, still-visible dark side (not a claim about real
  // earthshine/city-light brightness -- purely a "read as visible, not
  // pitch black" choice).
  constexpr float SCENE_AMBIENT_LIGHT = 0.35f;

  // Coordinate grid: three walls of a box (floor + two back walls) meeting
  // at a corner behind/below the satellite, acting as a visual coordinate
  // reference rather than an infinite ground plane. Half-size/step here
  // are the *pre-SATELLITE_VISUAL_SCALE* values (still proportioned
  // against the real 10cm cubesat/0.25m body-axis arrows); the call site
  // in main() applies SATELLITE_VISUAL_SCALE, same as every other drawn
  // satellite dimension.
  constexpr float GRID_HALF_SIZE = 3.0f;
  constexpr float GRID_STEP = 0.5f;

  // A visible marker radius for world-frame point markers (the TARGET
  // pointing mode's ground target) that, like the satellite, would
  // otherwise render as a sub-pixel dot at real orbital-distance camera
  // ranges -- unrelated to SATELLITE_VISUAL_SCALE (this is a mark on
  // Earth's real surface, not part of the satellite), sized to be
  // visible (a large-city-scale dot) against Earth's real 6.371e6m
  // radius rather than to represent any real object's true size.
  constexpr float TARGET_MARKER_RADIUS_M = 5.0e4f;

  // Ground footprint / ground-track minimum elevation: the horizon-limited
  // case (0 deg) shows the full circle a satellite can geometrically see
  // any part of -- a pure visualization of coverage geometry, deliberately
  // distinct from GROUND_STATION_MIN_ELEVATION_DEG below (a real station's
  // usable pass window, which wants a higher minimum to exclude low-
  // elevation passes with poor link geometry/obstruction).
  constexpr float FOOTPRINT_MIN_ELEVATION_DEG = 0.0f;

  // Minimum elevation for a ground-station contact to count as usable --
  // both for automatic target selection (GroundStations.h's
  // selectClosestGroundStation) and for the predicted pass schedule
  // (predictGroundStationPasses/GroundStationsPanel), so "this station is
  // the current target" and "this station appears as a valid pass" never
  // disagree. 10 deg is a typical real minimum (excludes near-horizon
  // passes, where atmospheric attenuation and terrain/building
  // obstruction usually make the link unusable) -- distinct from
  // FOOTPRINT_MIN_ELEVATION_DEG's pure horizon-limited geometry above.
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

  // Plot panel (world space, Y=-1.5 plane, X in [-0.5, 0.5], Z up)
  const glm::vec3 PLOT_ORIGIN{-0.5f, -1.5f, 0.0f};
  constexpr float PLOT_WIDTH = 1.0f;
  constexpr float PLOT_HEIGHT = 0.18f;
  constexpr float PLOT_GAP = 0.05f;

  constexpr float TUMBLE_KICK_RAD_S = 1.5f;

  // The ADCS task's own period (20 Hz) -- attitude estimation/guidance/
  // control/FDIR, the "control loop" rate. Also the fixed simulated-time
  // increment SimulationPanel's "Step" button advances by while paused.
  // Distinct from the individual sensor rates below and from
  // FIRMWARE_TICK_S, the fast tick FlightSoftware::step() itself is called
  // at -- see FswSchedule (src/fsw/FlightSoftwareHAL.h) for how these
  // combine into one multi-rate schedule.
  constexpr float TIME_STEP_S = 0.05f;

  // Per-sensor task periods for FswSchedule -- each representative of a
  // real cubesat-class component's own rate, not a fabricated number: a
  // MEMS IMU runs much faster than a star tracker's image-processing-bound
  // solve rate, and EPS/nav telemetry update far slower than either. Wheel
  // telemetry has no period of its own -- see FswSchedule's own comment.
  constexpr float IMU_PERIOD_S = 0.01f;          // 100 Hz
  constexpr float MAG_PERIOD_S = 0.05f;          // 20 Hz
  constexpr float STAR_TRACKER_PERIOD_S = 0.2f;  // 5 Hz
  constexpr float SUN_SENSOR_PERIOD_S = 0.1f;    // 10 Hz
  constexpr float POWER_PERIOD_S = 1.0f;         // 1 Hz
  constexpr float NAV_PERIOD_S = 1.0f;           // 1 Hz

  // The tick FlightSoftware::step() itself is called at -- must be <= the
  // fastest scheduled task period (IMU's, currently the fastest) so no
  // task's cadence is ever missed. The elapsed-time-accumulator pattern in
  // FlightSoftware::step() tolerates a tick that doesn't evenly divide
  // every period (it just fires a little late and keeps the remainder), so
  // exact divisibility isn't required, only tick <= fastest period.
  constexpr float FIRMWARE_TICK_S = IMU_PERIOD_S;

  constexpr int TELEMETRY_HISTORY_SAMPLES = 300;

  // Spacecraft mass/cross-section for the truth propagator's atmospheric
  // drag and solar radiation pressure force models -- matches
  // buildCubesatPyramid()'s real 1U body (0.1m cube, 1.33kg max mass).
  // Duplicated here rather than read off `sat.body` because the orbit
  // force models are constructed once at startup, before the mission
  // loop that could otherwise keep them in sync with a changing body.
  constexpr double SPACECRAFT_MASS_KG = 1.33;
  constexpr double SPACECRAFT_CROSS_SECTION_M2 = 0.1 * 0.1; // one 10x10cm face

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

  // A flat mirror mounted on the +Z face (same axis every pointing mode
  // aims), just for visualizing the sun-reflection geometry -- not wired
  // into ADCS/guidance at all.
  const glm::vec3 MIRROR_SIZE{0.09f, 0.09f, 0.003f}; // thin along its own normal (local Z)
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
  constexpr float POWER_STAR_TRACKER_W = 0.35f; // star trackers are the power-hungriest sensor on a cubesat bus
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
}
