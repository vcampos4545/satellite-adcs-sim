#include <vgl/vgl.h>
#include <rigidbody/PhysicsWorld.h>
#include <rigidbody/environment/central_body/CentralBodyMagneticField.h>
#include <rigidbody/orbit/OrbitState.h>
#include <rigidbody/orbit/OrbitalElements.h>
#include <rigidbody/orbit/OrbitForceModel.h>
#include <rigidbody/orbit/OrbitPropagator.h>
#include <rigidbody/orbit/OrbitFrames.h>
#include <rigidbody/orbit/OrbitTime.h>
#include <rigidbody/orbit/SunModel.h>
#include <rigidbody/orbit/MoonModel.h>
#include <rigidbody/orbit/EclipseModel.h>
#include <rigidbody/orbit/ThirdBodyGravity.h>
#include <rigidbody/orbit/AtmosphericDrag.h>
#include <rigidbody/orbit/SolarRadiationPressure.h>
#include <rigidbody/power/SolarPanel.h>
#include "fsw/FlightSoftware.h"
#include "fsw/FlightTypes.h"
#include "ImGuiLayer.h"
#include "Config.h"
#include "Cubesat.h"
#include "SensorTelemetry.h"
#include "GroundStations.h"
#include "rendering/SatelliteRenderer.h"
#include "rendering/MagneticFieldRenderer.h"
#include "rendering/OrbitRenderer.h"
#include "rendering/WorldAxesGizmo.h"
#include "panels/SimulationPanel.h"
#include "panels/VisualizationPanel.h"
#include "panels/GroundStationsPanel.h"
#include "panels/ADCSPanel.h"
#include "panels/FswPanel.h"
#include <random>
#include <memory>
#include <cmath>
#include <cstdio>

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main()
{
  GUI gui(800, 600, "Satellite Simulator");
  ImGuiLayer imguiLayer(gui);
  gui.camera
      .setUp({0, 0, 1})
      .setClipPlanes(Config::CAMERA_NEAR, Config::CAMERA_FAR)
      .setFOV(Config::CAMERA_FOV);
  // The scene now spans a 0.1m cubesat up to a ~6.9e6m orbital radius --
  // a standard depth buffer can't hold that dynamic range without
  // z-fighting (near:far is on the order of 1e9). Logarithmic depth
  // trades a small amount of precision at extreme distances for correct
  // ordering across the whole range.
  gui.setLogDepth(Config::CAMERA_FAR);
  // Raised from VGL's default 0.15 -- Earth's night side otherwise reads
  // as near-black, harsher than the visual reference this project is
  // going for (not a claim about real earthshine/city-light brightness,
  // just a softer unlit floor). See Config::SCENE_AMBIENT_LIGHT.
  gui.setAmbientLight(Config::SCENE_AMBIENT_LIGHT);

  // Loaded after the GL context is live (Texture::loadFromFile requires
  // it) -- falls back to a grey 1x1 texture with a printed warning if the
  // file isn't found, so a missing asset doesn't crash the sim.
  Texture earthTexture;
  earthTexture.loadFromFile(std::string(RESOURCES_DIR) + "/textures/earth.jpg");
  Texture sunTexture;
  sunTexture.loadFromFile(std::string(RESOURCES_DIR) + "/textures/sun.jpg");
  Texture moonTexture;
  moonTexture.loadFromFile(std::string(RESOURCES_DIR) + "/textures/moon.jpg");

  PhysicsWorld world;
  HardwareConfig hwConfig;
  Cubesat sat = buildCubesatPyramid(world, hwConfig);

  // Built after the satellite so the orbit target can start on its actual
  // position rather than a magic-number guess at where buildCubesatPyramid()
  // happens to place it.
  OrbitalCamera orbit(
      Config::CAMERA_INITIAL_DISTANCE,
      45.0f, 0.0f,
      sat.body->position);
  orbit.setMaxDistance(Config::CAMERA_MAX_DISTANCE)
      .setMinDistance(Config::CAMERA_MIN_DISTANCE)
      .setZoomSensitivity(Config::ZOOM_SENSITIVITY)
      .setPanSensitivity(Config::PAN_SENSITIVITY);

  glm::vec2 lastMousePos = gui.getMousePosition();

  // Flight software: FlightSoftware is this project's actual cyclic FSW
  // entry point (see src/fsw/FlightSoftware.h) -- owns ADCS (attitude
  // estimation/guidance/control/allocation) and FDIR (fault detection/
  // mode override) as peers, and is hardware-abstracted the same way ADCS
  // itself is (see FlightTypes.h): configure() gives it a fixed hardware
  // description and an initial attitude estimate; every cycle after this,
  // step() only ever sees plain FSWInputs this loop builds from `sat`
  // (Cubesat::sampleSensors(), the one place simulated hardware is
  // translated to/from FSW's plain-data contract) and returns plain
  // FSWOutputs this loop applies via `sat.applyActuatorCommands()`.
  // Neither FlightSoftware nor ADCS ever holds a RigidBody*/sensor
  // pointer/actuator pointer at all. `adcs` stays a local reference into
  // it so the rest of this file's existing adcs.foo calls (mode, target,
  // telemetry fields, ...) don't need to change.
  FlightSoftware flightSoftware;
  ADCS &adcs = flightSoftware.adcs;
  flightSoftware.configure(hwConfig, sat.body->orientation);
  // adcs.target is set below, once orbitState/missionEpochJd exist -- see
  // the ground-station selection block after they're established.
  float fswTimer = 0.0f;
  float missionTime = 0.0f;
  float trueErrDeg = 0.0f; // harness-side diagnostic: true (ground-truth) pointing error, held between ADCS cycles like fieldNow below

  // No Gravity generator is attached to this world (free-floating orbit),
  // so ambient gravity for the IMU model is zero -- this is harness-side
  // simulation setup now, not something ADCS itself ever sees or assumes.
  glm::vec3 gravity{0.0f};

  // Real orbital truth: double-precision ECI state (see rigidbody/orbit/
  // OrbitState.h), propagated independently of sat.body's own float32
  // scene-frame position -- this is genuinely a *different* position, not
  // a rescaled version of it (see FlightTypes.h's spacecraftPositionWorld
  // comment). Two-body + J2, ISS-like default (500km circular, 51.6deg
  // inclination) -- the same altitude/inclination the old kinematic
  // magnetic-field stand-in used, now driving real position, sun
  // direction, eclipse state, and magnetic field.
  OrbitState orbitState = OrbitalElements::circular(
                              500e3, glm::radians(51.6), OrbitFrames::EARTH_RADIUS_M)
                              .toState(TwoBodyGravity{}.mu);
  OrbitPropagator orbitPropagator;
  orbitPropagator.addForceModel(std::make_unique<TwoBodyGravity>());
  orbitPropagator.addForceModel(std::make_unique<J2Perturbation>());
  orbitPropagator.addForceModel(
      std::make_unique<AtmosphericDrag>(Config::SPACECRAFT_CROSS_SECTION_M2, Config::SPACECRAFT_MASS_KG));

  // ThirdBodyGravity/SolarRadiationPressure need epochJd refreshed every
  // frame (see their own header comments) since this project's mission
  // epoch is live-editable from the Simulation tab -- keep raw observing
  // pointers into the propagator's owned models rather than reconstructing
  // them, so OrbitPropagator retains sole ownership.
  auto sunGravityOwned = std::make_unique<ThirdBodyGravity>(ThirdBodyType::Sun);
  auto moonGravityOwned = std::make_unique<ThirdBodyGravity>(ThirdBodyType::Moon);
  auto srpOwned = std::make_unique<SolarRadiationPressure>(Config::SPACECRAFT_CROSS_SECTION_M2, Config::SPACECRAFT_MASS_KG);
  ThirdBodyGravity *sunGravity = sunGravityOwned.get();
  ThirdBodyGravity *moonGravity = moonGravityOwned.get();
  SolarRadiationPressure *srp = srpOwned.get();
  orbitPropagator.addForceModel(std::move(sunGravityOwned));
  orbitPropagator.addForceModel(std::move(moonGravityOwned));
  orbitPropagator.addForceModel(std::move(srpOwned));

  EpochControls epoch; // editable live from the Simulation tab -- see its own comment
  double missionEpochJd = OrbitTime::julianDate(epoch.year, epoch.month, epoch.day, epoch.hour, epoch.minute, epoch.second);
  sunGravity->epochJd = missionEpochJd;
  moonGravity->epochJd = missionEpochJd;
  srp->epochJd = missionEpochJd;

  // Real tilted-dipole field, sampled at orbitState's true position each
  // frame (see below) -- replaces the old MagneticField's fake kinematic
  // orbit phase with the real one. Written into both the magnetorquers
  // (which need it to turn a commanded dipole moment into torque) and
  // ADCS's ambientFieldWorld (its TRIAD reference vector), the same way
  // `gravity` feeds the IMU sample below.
  CentralBodyMagneticField magField;

  // Global field-line geometry for drawMagneticFieldLines -- traced once
  // since this model's dipole is fixed-inertial (see traceDipoleFieldLines'
  // own comment), not per frame.
  std::vector<FieldLine> magneticFieldLines = traceDipoleFieldLines(magField);

  SensorTelemetry telemetry(Config::TELEMETRY_HISTORY_SAMPLES);

  SimControls sim(Config::TUMBLE_KICK_RAD_S);
  VisualizationSettings vis;                // scene-element toggles -- see its own header comment for defaults
  glm::vec3 fieldNow{0.0f};                 // last-sampled ambient field; held while paused rather than resampled
  bool inEclipse = false;                   // last-computed shadow state; held while paused
  glm::vec3 earthRelativePositionNow{0.0f}; // orbitState.position cast to float; held while paused
  glm::vec3 moonPositionNow{0.0f};          // MoonModel::positionEci cast to float; held while paused

  // Predicted orbit path (render-scale points, see computePredictedOrbitPath)
  // -- computed once up front and refreshed periodically, not every frame.
  std::vector<glm::vec3> orbitPathPoints = computePredictedOrbitPath(orbitState, Config::ORBIT_PATH_POINTS, missionEpochJd);
  float orbitPathRefreshTimer = 0.0f;
  double currentJdNow = missionEpochJd; // held while paused, and for drawEarth() below

  // Ground track: (lat, lon) history in degrees, sampled periodically (see
  // Config::GROUND_TRACK_SAMPLE_INTERVAL_S) rather than every frame, capped
  // at Config::GROUND_TRACK_MAX_POINTS so it spans multiple orbits without
  // growing unbounded over a long-running mission.
  std::vector<glm::vec2> groundTrackLatLonDeg;
  float groundTrackSampleTimer = 0.0f;

  // adcs.target auto-tracks the closest ground station within the
  // satellite's footprint (elevation >= Config::GROUND_STATION_MIN_ELEVATION_DEG
  // -- the same threshold the predicted pass schedule below uses, so
  // "this is the current target" and "this station appears as a valid
  // pass" never disagree). A station outside the footprint isn't a
  // viable target at all -- selectClosestGroundStation returns nullptr
  // rather than falling back to an unreachable one, and when it does,
  // adcs.target falls back to the real Sun position instead (see the
  // reselection block inside the main loop below). selectedGroundStation
  // is compared by address each frame (stable, since it always points
  // into the static GROUND_STATIONS array, or is nullptr) purely to know
  // when the *selection itself* changed, so resetController() (clears
  // integral windup) only fires on an actual handoff -- a ground-station-
  // to-ground-station handoff, a station-to-Sun fallback, or back --
  // not every frame the target's rotating ECI position moves.
  glm::dvec3 initialTargetEci;
  const GroundStation *selectedGroundStation = selectClosestGroundStation(
      orbitState.position, OrbitFrames::gmstRad(missionEpochJd),
      glm::radians(static_cast<double>(Config::GROUND_STATION_MIN_ELEVATION_DEG)), initialTargetEci);
  adcs.target = selectedGroundStation ? glm::vec3(initialTargetEci)
                                      : glm::vec3(SunModel::positionEci(missionEpochJd));

  // Predicted ground-station contact schedule (Ground Stations tab) --
  // computed once up front and refreshed periodically. Unlike
  // orbitPathRefreshTimer above, this accumulates real wall-clock dt, not
  // simDt (see Config::PASS_PREDICTION_REFRESH_S's own comment): a 24h/
  // 15s-step prediction is real work (thousands of RK4 steps), and tying
  // its cadence to simDt would make it run more often -- not less -- the
  // faster SimControls::timeScale is turned up, exactly backwards from
  // what keeping the frame rate steady at high time-scale needs.
  std::vector<GroundStationPass> groundStationPasses = predictGroundStationPasses(
      orbitState, missionEpochJd, glm::radians(static_cast<double>(Config::GROUND_STATION_MIN_ELEVATION_DEG)),
      Config::PASS_PREDICTION_LOOKAHEAD_S, Config::PASS_PREDICTION_STEP_S);
  float groundStationPassRefreshTimer = 0.0f;
  int selectedPassIndex = -1; // Ground Stations tab's selected table row; -1 = none

  float lastTime = glfwGetTime();
  while (!gui.shouldClose())
  {
    float time = glfwGetTime();
    float dt = time - lastTime;
    lastTime = time;

    // =================== INPUT ===================
    glm::vec2 mousePos = gui.getMousePosition();
    glm::vec2 mouseDelta = mousePos - lastMousePos;
    lastMousePos = mousePos;

    // Follow the satellite
    orbit.setTarget(sat.body->position);

    // Check ImGUi
    if (!ImGui::GetIO().WantCaptureMouse)
      orbit.handleInput(gui, mouseDelta, gui.getScrollDelta());
    orbit.applyToCamera(gui.camera);

    // =================== SIMULATION ===================
    bool advanceSim = !sim.paused;
    float simDt = dt * sim.timeScale;
    if (sim.paused && sim.stepRequested)
    {
      advanceSim = true;
      simDt = Config::TIME_STEP_S;
      sim.stepRequested = false;
    }

    if (advanceSim)
    {
      missionTime += simDt;

      // One fixed-rate loop drives orbit propagation, PhysicsWorld::step(),
      // and one FlightSoftware cycle together, in that order, matching how
      // a real sampled control loop is discretized: physics integrates
      // using whatever actuator commands were applied at the *end* of the
      // previous cycle (zero-order hold), then FSW reads the freshly-
      // integrated state and computes the next command. A `while`, not an
      // `if`: at timeScale > 1x, simDt can span more than one nominal
      // Config::TIME_STEP_S cycle in a single render frame -- an `if`
      // would silently run everything at a *slower relative rate* the
      // faster time is scaled (fewer corrections per orbit, and fewer
      // physics substeps), which would visibly degrade pointing/detumble
      // stability at high speeds for no physical reason. Catching up with
      // a fixed-size `while` keeps every system's cadence correct relative
      // to simulated time regardless of timeScale, the same fixed-step-
      // accumulator pattern PhysicsWorld::step() already uses internally
      // one level down.
      fswTimer += simDt;
      while (fswTimer > Config::TIME_STEP_S)
      {
        fswTimer -= Config::TIME_STEP_S;

        // =================== ORBIT (translational truth) ===================
        // RK4 is stable at any step size -- smaller just means more calls,
        // not less accuracy -- integrated independently of
        // PhysicsWorld::step() below in double precision, then bridged
        // into sat.body->position as a single non-accumulating cast every
        // cycle (see OrbitState.h's header comment for why the double-
        // precision integration itself has to stay separate from
        // RigidBody's float32 state, even though the *result* is copied
        // into it here). Because this is a fresh cast from the true state
        // each cycle rather than something PhysicsWorld itself integrates,
        // there's no float32 accumulation risk -- sat.body->velocity is
        // deliberately never set to match, so PhysicsWorld's own (float32)
        // translational integration contributes nothing on top of this.
        orbitPropagator.step(orbitState, Config::TIME_STEP_S);
        // Recomputed from the Simulation tab's live-editable fields every
        // cycle -- cheap (six ints/one float into a JD formula), and means
        // an edit takes effect immediately without a separate "apply" step.
        missionEpochJd = OrbitTime::julianDate(epoch.year, epoch.month, epoch.day, epoch.hour, epoch.minute, epoch.second);
        sunGravity->epochJd = missionEpochJd;
        moonGravity->epochJd = missionEpochJd;
        srp->epochJd = missionEpochJd;
        currentJdNow = OrbitTime::advance(missionEpochJd, orbitState.missionTimeS);
        earthRelativePositionNow = glm::vec3(orbitState.position); // single non-accumulating cast -- see OrbitState.h
        sat.body->position = earthRelativePositionNow;
        glm::dvec3 sunDirEci = SunModel::directionEci(currentJdNow);
        inEclipse = EclipseModel::inEclipse(orbitState.position, sunDirEci);
        moonPositionNow = glm::vec3(MoonModel::positionEci(currentJdNow)); // single non-accumulating cast, same as earthRelativePositionNow

        // Refresh the predicted-path polyline periodically (not every
        // cycle -- see ORBIT_PATH_REFRESH_S) since it only drifts slowly
        // (mainly from J2) cycle to cycle.
        orbitPathRefreshTimer += Config::TIME_STEP_S;
        if (orbitPathRefreshTimer > Config::ORBIT_PATH_REFRESH_S)
        {
          orbitPathPoints = computePredictedOrbitPath(orbitState, Config::ORBIT_PATH_POINTS, missionEpochJd);
          orbitPathRefreshTimer = 0.0f;
        }

        // Sample the ground track periodically (not every cycle -- see
        // GROUND_TRACK_SAMPLE_INTERVAL_S).
        groundTrackSampleTimer += Config::TIME_STEP_S;
        if (groundTrackSampleTimer > Config::GROUND_TRACK_SAMPLE_INTERVAL_S)
        {
          OrbitFrames::Geodetic geo = OrbitFrames::eciToGeodeticDeg(orbitState.position, OrbitFrames::gmstRad(currentJdNow));
          groundTrackLatLonDeg.push_back(glm::vec2(static_cast<float>(geo.latDeg), static_cast<float>(geo.lonDeg)));
          if (static_cast<int>(groundTrackLatLonDeg.size()) > Config::GROUND_TRACK_MAX_POINTS)
            groundTrackLatLonDeg.erase(groundTrackLatLonDeg.begin());
          groundTrackSampleTimer = 0.0f;
        }

        adcs.sunPosition = glm::vec3(SunModel::positionEci(currentJdNow));

        // Ground-station target selection: recomputed every FSW cycle
        // (cheap -- 6 stations, a distance/elevation compare each), so
        // adcs.target keeps tracking the selected station's real rotating
        // ECI position even between actual handoffs. A station must be
        // within the satellite's footprint (elevation >=
        // Config::GROUND_STATION_MIN_ELEVATION_DEG) to be a viable target
        // at all -- selectClosestGroundStation returns nullptr rather than
        // falling back to an out-of-view station, in which case
        // adcs.targetValid goes false and ADCS's own guidance falls back
        // to sun-relative pointing for TARGET/SLEW/FINE_POINTING/REFLECT
        // (see ADCS.h's targetValid comment) -- the typical real-ADCS
        // convention for "this mode's reference isn't available right
        // now," rather than this harness spoofing `target` itself to point
        // somewhere else. adcs.target is simply left stale (harmless --
        // unread while !targetValid) when no station is selected. Only an
        // actual change of *what's* selected (a different station, or the
        // Sun fallback engaging/disengaging) clears controller integral
        // windup -- the position update itself is continuous, not a
        // discrete retarget the controller needs to react to as one.
        glm::dvec3 targetEci;
        const GroundStation *chosenGroundStation = selectClosestGroundStation(
            orbitState.position, OrbitFrames::gmstRad(currentJdNow),
            glm::radians(static_cast<double>(Config::GROUND_STATION_MIN_ELEVATION_DEG)), targetEci);
        adcs.targetValid = (chosenGroundStation != nullptr);
        if (chosenGroundStation)
          adcs.target = glm::vec3(targetEci);
        if (chosenGroundStation != selectedGroundStation)
        {
          selectedGroundStation = chosenGroundStation;
          adcs.resetController(); // clear integral windup from the previous target
        }

        // Ambient field at the satellite's real orbital position -- fed to
        // the magnetorquers (they need it every physics substep to turn a
        // commanded dipole moment into torque), to ADCS (it needs it to
        // interpret the magnetometer), and to `sat` itself (its own
        // sensor-sampling methods need it -- see Cubesat::sampleSensors()).
        fieldNow = magField.sample(earthRelativePositionNow);
        for (auto *rod : sat.magnetorquers)
          rod->ambientFieldWorld = fieldNow;
        adcs.ambientFieldWorld = fieldNow;

        glm::vec3 sunDirWorld = adcs.sunPosition - sat.body->position;
        sat.gravity = gravity;
        sat.ambientFieldWorld = fieldNow;
        sat.sunDirWorld = sunDirWorld;
        sat.inEclipse = inEclipse;

        // =================== PHYSICS ===================
        // Rotational dynamics: integrates the wheel/magnetorquer commands
        // applied at the end of the *previous* cycle (zero-order hold --
        // see this loop's own header comment above).
        world.step(Config::TIME_STEP_S);

        // =================== FLIGHT SOFTWARE ===================
        // The only place simulated hardware is translated to/from FSW's
        // plain-data contract -- see Cubesat::sampleSensors()/
        // applyActuatorCommands() and FlightSoftware.h's own header
        // comment. FlightSoftware::step() itself never touches `sat`.
        FSWInputs in = sat.sampleSensors(Config::TIME_STEP_S);
        FSWOutputs out = flightSoftware.step(in, Config::TIME_STEP_S);
        sat.applyActuatorCommands(out);

        // =================== EPS ===================
        // Generation: sum every panel's cosine-law output against the same
        // sun direction the star tracker/sun sensor just read against --
        // zero while the real orbital position is in Earth's shadow (see
        // EclipseModel::inEclipse above), closing this project's former "no
        // orbital eclipse model" gap. Consumption: a fixed housekeeping/
        // sensor draw plus each actuator's idle-plus-effort power for the
        // commands just issued above -- see the Config::POWER_* comments
        // for the model each term follows. Net power integrates straight
        // into the battery.
        float genW = 0.0f;
        if (!inEclipse)
          for (const SolarPanel &panel : sat.solarPanels)
            genW += panel.sample(*sat.body, sunDirWorld, Config::SOLAR_FLUX_WM2).powerW;

        float drawW = Config::POWER_OBC_BASELINE_W + Config::POWER_IMU_W +
                      Config::POWER_MAGNETOMETER_W + Config::POWER_STAR_TRACKER_W +
                      Config::POWER_SUN_SENSOR_W;
        for (int i = 0; i < NUM_WHEELS; i++)
          drawW += Config::WHEEL_IDLE_POWER_W +
                   std::abs(out.wheelCommands[i].torqueNm * sat.wheels[i]->currentSpeed) / Config::WHEEL_MOTOR_EFFICIENCY;
        for (int i = 0; i < NUM_TORQUERS; i++)
          drawW += Config::TORQUER_IDLE_POWER_W +
                   std::abs(out.torquerCommands[i].momentAm2) * Config::TORQUER_POWER_PER_AM2_W;

        sat.battery.update(genW - drawW, Config::TIME_STEP_S);
        telemetry.netPowerW.push(genW - drawW);

        // Ground-truth diagnostic -- computed here, not by ADCS (which has
        // no way to know body->orientation at all anymore), the same way
        // this project's earlier true-vs-estimated pointing error
        // comparisons always required direct simulation access.
        glm::quat trueErrQ = glm::inverse(sat.body->orientation) * adcs.targetAttitude;
        if (trueErrQ.w < 0.0f)
          trueErrQ = -trueErrQ;
        trueErrDeg = glm::degrees(2.0f * std::acos(glm::clamp(trueErrQ.w, -1.0f, 1.0f)));

        // Pushed once per FSW cycle (a new sensor reading actually
        // exists), not once per render frame.
        telemetry.gyroMagDegS.push(glm::degrees(glm::length(adcs.lastGyroBody)));
        telemetry.accelMagMs2.push(glm::length(adcs.lastAccelBody));
        telemetry.magFieldMagUt.push(glm::length(adcs.magFieldBody) * 1e6f);
        telemetry.estimatedPointingErrorDeg.push(adcs.estimatedPointingErrorDeg);
        telemetry.truePointingErrorDeg.push(trueErrDeg);
        telemetry.batterySocPct.push(sat.battery.stateOfCharge() * 100.0f);
      }
    }

    // Ground-station pass schedule: refreshed on a real wall-clock timer
    // (see groundStationPassRefreshTimer's own declaration comment above)
    // regardless of pause state -- recomputing while paused just re-runs
    // the same prediction from the frozen orbitState, harmless, and keeps
    // the schedule live if the user edits the mission epoch while paused.
    groundStationPassRefreshTimer += dt;
    if (groundStationPassRefreshTimer > Config::PASS_PREDICTION_REFRESH_S)
    {
      groundStationPasses = predictGroundStationPasses(
          orbitState, missionEpochJd, glm::radians(static_cast<double>(Config::GROUND_STATION_MIN_ELEVATION_DEG)),
          Config::PASS_PREDICTION_LOOKAHEAD_S, Config::PASS_PREDICTION_STEP_S);
      groundStationPassRefreshTimer = 0.0f;
    }

    // =================== DRAW ===================
    // VGL's scene lighting is a single directional light (see GUI.h's
    // m_lightDir / EmbeddedShaders.h's diffuse term), defaulting to an
    // arbitrary fixed world-space direction -- it has no idea where the
    // real Sun is unless told every frame. `lightDir` is the direction
    // *toward* the light (the shader does `dot(normal, normalize(lightDir))`),
    // so this must be the direction from Earth's center toward the Sun,
    // not the Sun's position itself -- without this call Earth's lit
    // hemisphere is disconnected from where the Sun marker is actually
    // drawn, which is what made the sunlit side look inverted.
    gui.setLightDirection(glm::normalize(adcs.sunPosition));
    gui.beginFrame();
    imguiLayer.beginFrame();

    // =================== ORBIT VISUALIZATION ===================
    // Earth and the predicted orbit path, both in real ECI meters --
    // PhysicsWorld's own frame here *is* ECI (Earth's center at the world
    // origin, sat.body->position bridged from orbitState every frame
    // below), so the satellite's own wireframe/wheels/rods/etc. (drawn
    // further down, already reading sat.body->position/orientation) are
    // already at the right place -- no separate marker needed.
    if (vis.showEarth)
      drawEarth(gui, earthTexture, currentJdNow);
    if (vis.showSun)
    {
      float sunRadius = static_cast<float>(OrbitFrames::SUN_RADIUS_M);
      if (sunTexture.isLoaded())
        gui.drawTexturedSphere(adcs.sunPosition, sunRadius, TEXTURED_SPHERE_POLE_ALIGNMENT, sunTexture, /*unlit=*/true);
      else
        gui.drawSphere(adcs.sunPosition, sunRadius, {1.0f, 0.9f, 0.1f});
    }
    if (vis.showMoon)
    {
      if (moonTexture.isLoaded())
        gui.drawTexturedSphere(moonPositionNow, static_cast<float>(OrbitFrames::MOON_RADIUS_M), TEXTURED_SPHERE_POLE_ALIGNMENT, moonTexture);
      else
        gui.drawSphere(moonPositionNow, static_cast<float>(OrbitFrames::MOON_RADIUS_M), {0.75f, 0.75f, 0.78f});
    }
    if (vis.showOrbitPath)
      drawOrbitPath(gui, orbitPathPoints);
    if (vis.showGroundFootprint)
      drawGroundFootprint(gui, orbitState.position, glm::radians(Config::FOOTPRINT_MIN_ELEVATION_DEG));
    if (vis.showGroundStations)
      drawGroundStations(gui, OrbitFrames::gmstRad(currentJdNow));

    if (vis.showSatellite)
    {
      drawSatelliteWireframe(gui, sat.body);
      drawReactionWheels(gui, sat.wheels, sat.body);
      drawMagnetorquers(gui, sat.magnetorquers, sat.body);
      drawMirror(gui, sat.body);
      if (adcs.mode == PointingMode::REFLECT)
        drawSunReflection(gui, sat.body, adcs.sunPosition);
    }
    if (vis.showMagneticField)
    {
      drawMagneticField(gui, fieldNow, sat.body->position);
      drawMagneticFieldLines(gui, magneticFieldLines);
    }

    if (vis.showTargetMarker)
      gui.drawSphere(adcs.target, Config::TARGET_MARKER_RADIUS_M, {0, 1.0f, 0});

    // Target line
    if (vis.showTargetMarker)
      gui.drawLine(sat.body->position, adcs.target, {0, 1.0f, 0});

    if (vis.showWorldAxesGizmo)
      drawWorldAxesGizmo(gui);

    drawADCSPanel(flightSoftware, sat, telemetry, sim, epoch, vis, orbitState, earthTexture, groundTrackLatLonDeg, currentJdNow,
                  groundStationPasses, selectedPassIndex, trueErrDeg, inEclipse);

    imguiLayer.endFrame();
    gui.endFrame();
  }
  return 0;
}
