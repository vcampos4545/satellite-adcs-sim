#pragma once
#include <glm/glm.hpp>

// Rendering/camera/UI knobs -- things that change what's drawn, or how
// history/prediction windows are sized, never simulated behavior itself.
// See SatelliteConfig.h for this project's spacecraft design/FSW timing,
// and PhysicalConstants.h for universal physics/astronomy.
namespace VisualizationConfig
{
  // ---------------------------------------------------------------------
  // Scene
  // ---------------------------------------------------------------------
  constexpr float SATELLITE_VISUAL_SCALE = 550.0f;
  constexpr float SCENE_AMBIENT_LIGHT = 0.35f;
  constexpr float TARGET_MARKER_RADIUS_M = 5.0e4f;
  constexpr float GROUND_MARKER_RADIUS_M = 3.5e4f; // ground stations + solar farms -- smaller than TARGET_MARKER_RADIUS_M so the selected/green target still reads as primary

  // ---------------------------------------------------------------------
  // Camera
  // ---------------------------------------------------------------------
  constexpr float CAMERA_NEAR = 10.0f;
  constexpr float CAMERA_FAR = 2.0e11f;
  constexpr float CAMERA_FOV = 45.0f;
  constexpr float SATELLITE_HALF_DIAGONAL_M = 12.7279f; // 0.5*sqrt(18^2 + 18^2), real (pre-scale) mirror half-diagonal
  constexpr float CAMERA_MIN_DISTANCE = SATELLITE_HALF_DIAGONAL_M * SATELLITE_VISUAL_SCALE * 2.0f;
  constexpr float CAMERA_INITIAL_DISTANCE = CAMERA_MIN_DISTANCE * 1.6f;
  constexpr float CAMERA_MAX_DISTANCE = 1.0e8f;
  constexpr float ZOOM_SENSITIVITY = 1.0f;
  constexpr float PAN_SENSITIVITY = 0.2f;

  // ---------------------------------------------------------------------
  // Ground footprint / station targeting display
  // ---------------------------------------------------------------------
  constexpr float FOOTPRINT_MIN_ELEVATION_DEG = 0.0f; // horizon-limited -- see docs/ALGORITHMS.md's "Guidance" section
  constexpr int FOOTPRINT_CIRCLE_SEGMENTS = 96;
  constexpr int FOOTPRINT_NADIR_DASH_SEGMENTS = 20;

  // Ground-station pass prediction (predictGroundStationPasses): how far
  // ahead to search, and the fixed time step the AOS/LOS/max-elevation
  // search advances by. A LEO pass typically lasts 5-15 minutes, so a 15s
  // step gives tens of samples per pass (AOS/LOS timing resolution is
  // +/- one step) without making a 24h search expensive. Recomputed on a
  // real (wall-clock, not simulated) timer -- see PASS_PREDICTION_REFRESH_S
  // -- so cost stays constant regardless of SimControls::timeScale, unlike
  // the orbit path's own simDt-driven refresh.
  constexpr double PASS_PREDICTION_LOOKAHEAD_S = 24.0 * 3600.0;
  constexpr double PASS_PREDICTION_STEP_S = 15.0;
  constexpr float PASS_PREDICTION_REFRESH_S = 30.0f;

  // ---------------------------------------------------------------------
  // Ground track: how much history to keep and how often to sample it.
  // Sampling once every GROUND_TRACK_SAMPLE_INTERVAL_S of mission time
  // (not every frame) at GROUND_TRACK_MAX_POINTS keeps the trail spanning
  // multiple orbits without growing unbounded over a long-running mission.
  // ---------------------------------------------------------------------
  constexpr int GROUND_TRACK_MAX_POINTS = 400;
  constexpr float GROUND_TRACK_SAMPLE_INTERVAL_S = 15.0f;

  // ---------------------------------------------------------------------
  // Orbit path prediction: PhysicsWorld's own coordinate frame *is* ECI --
  // Earth's center is the world origin, and sat.body->position is bridged
  // from orbitState every frame, so there's no separate render scale/
  // offset to track.
  // ---------------------------------------------------------------------
  constexpr int ORBIT_PATH_POINTS = 120;       // segments in the predicted-path polyline
  constexpr float ORBIT_PATH_REFRESH_S = 5.0f; // real seconds between path recomputes

  // ---------------------------------------------------------------------
  // Telemetry plots
  // ---------------------------------------------------------------------
  constexpr int TELEMETRY_HISTORY_SAMPLES = 300;

  // ---------------------------------------------------------------------
  // Global dipole field-line visualization (see traceDipoleFieldLines):
  // seed colatitudes/azimuths (both hemispheres) for the traced loops, an
  // RK4 arclength step size, and a per-line point cap as a safety bound
  // against near-axis seeds (whose loops can be very large) never
  // reaching the closure/max-radius stopping condition -- plus the
  // ambient-field arrow's own visual scale and every field-line color.
  // ---------------------------------------------------------------------
  constexpr float FIELD_LINE_COLATITUDES_DEG[] = {15.0f, 30.0f, 45.0f};
  constexpr int FIELD_LINE_AZIMUTH_COUNT = 10;
  constexpr float FIELD_LINE_STEP_FRAC_EARTH_RADIUS = 0.03f; // RK4 step, as a fraction of Earth's radius
  constexpr int FIELD_LINE_MAX_POINTS = 200;
  constexpr float FIELD_LINE_MAX_RADIUS_FRAC_EARTH_RADIUS = 12.0f; // safety cutoff for open-looking loops
  constexpr float FIELD_VISUAL_SCALE = 8000.0f; // scales a ~20-60 uT LEO field into a visible arrow length
  const glm::vec3 FIELD_LINE_NORTH_COLOR{0.25f, 0.65f, 0.95f};
  const glm::vec3 FIELD_LINE_SOUTH_COLOR{0.95f, 0.65f, 0.15f};
  const glm::vec3 FIELD_ARROW_COLOR{0.2f, 0.9f, 0.9f};

  // ---------------------------------------------------------------------
  // Decorative bus box + mirror reflection ray. The real RigidBody *is*
  // the 18m x 18m mirror plate (see buildSatellite()), so its own +Z face
  // is the actual reflecting surface -- MIRROR_NORMAL_BODY still names
  // that axis for drawSunReflection()'s geometry. BUS_SIZE is purely a
  // decorative box mounted just behind the plate (-Z), representing the
  // small bus core sitting at the mirror's center -- not a second
  // RigidBody, not part of physics.
  // ---------------------------------------------------------------------
  const glm::vec3 BUS_SIZE{0.5f, 0.5f, 0.5f};
  const glm::vec3 MIRROR_NORMAL_BODY{0.0f, 0.0f, 1.0f};
  constexpr float REFLECTED_RAY_LENGTH = 1.0f;

  // ---------------------------------------------------------------------
  // Satellite wireframe / actuator visualization -- sizes as fractions of
  // SATELLITE_VISUAL_SCALE so they stay proportionate to the satellite
  // regardless of its scale, plus the wireframe/body-axis-arrow colors.
  // ---------------------------------------------------------------------
  const glm::vec3 WIREFRAME_COLOR{1.0f, 1.0f, 0.0f};
  constexpr float BODY_AXIS_ARROW_LENGTH_FRAC = 0.25f;
  constexpr float WHEEL_VIS_RADIUS_FRAC = 0.02f;
  constexpr float WHEEL_VIS_THICKNESS_FRAC = 0.006f;
  constexpr float WHEEL_VIS_ARROW_LENGTH_FRAC = 0.05f;
  constexpr float TORQUER_VIS_ROD_RADIUS_FRAC = 0.006f;
  constexpr float TORQUER_VIS_ROD_LENGTH_FRAC = 0.035f;
  constexpr float TORQUER_VIS_ARROW_LENGTH_FRAC = 0.06f;

  // ---------------------------------------------------------------------
  // Orbit/ground overlays
  // ---------------------------------------------------------------------
  const glm::vec3 ORBIT_PATH_COLOR{0.3f, 0.7f, 1.0f};
  const glm::vec3 FOOTPRINT_COLOR{0.0f, 0.85f, 1.0f};
  const glm::vec3 GROUND_STATION_COLOR{0.85f, 0.85f, 0.9f};
  const glm::vec3 SOLAR_FARM_COLOR{0.95f, 0.75f, 0.15f}; // gold, distinct from ground stations' gray

  // ---------------------------------------------------------------------
  // World-axes gizmo (bottom-left orientation compass)
  // ---------------------------------------------------------------------
  constexpr float AXES_GIZMO_MARGIN_PX = 80.0f; // pixels from the corner
  constexpr float AXES_GIZMO_ARM_PX = 50.0f;    // pixel length of each fully-foreshortened axis
}
